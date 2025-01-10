#include "stdafx.h"
#include "SoundRender_Source.h"
#include "SoundRender_Core.h"

#include <vorbis/vorbisfile.h>

namespace
{
bool ov_can_continue_read(long res)
{
    switch (res)
    {
    case 0:
        return false;
    // info
    case OV_HOLE:
        Msg("Vorbisfile encoutered missing or corrupt data in the bitstream. Recovery is normally automatic and this "
            "return code is for informational purposes only.");
        return true;
    case OV_EBADLINK:
        Msg("The given link exists in the Vorbis data stream, but is not decipherable due to garbage or corruption.");
        return true;
    // error
    case OV_FALSE: Msg("Not true, or no data available"); return false;
    case OV_EREAD: Msg("Read error while fetching compressed data for decode"); return false;
    case OV_EFAULT: Msg("Internal inconsistency in decode state. Continuing is likely not possible."); return false;
    case OV_EIMPL: Msg("Feature not implemented"); return false;
    case OV_EINVAL:
        Msg("Either an invalid argument, or incompletely initialized argument passed to libvorbisfile call");
        return false;
    case OV_ENOTVORBIS: Msg("The given file/data was not recognized as Ogg Vorbis data."); return false;
    case OV_EBADHEADER:
        Msg("The file/data is apparently an Ogg Vorbis stream, but contains a corrupted or undecipherable header.");
        return false;
    case OV_EVERSION: Msg("The bitstream format revision of the given stream is not supported."); return false;
    case OV_ENOSEEK: Msg("The given stream is not seekable"); return false;
    }
    return false;
}

constexpr ov_callbacks g_ov_callbacks = {
    // read
    [](void* ptr, size_t size, size_t nmemb, void* datasource) -> size_t {
        auto* file = static_cast<IReader*>(datasource);
        const size_t exist_block = _max(0ul, iFloor(file->elapsed() / (float)size));
        const size_t read_block = std::min(exist_block, nmemb);
        file->r(ptr, read_block * size);
        return read_block;
    },
    // seek
    [](void* datasource, ogg_int64_t offset, int whence) -> int {
        // SEEK_SET 0 File beginning
        // SEEK_CUR 1 Current file pointer position
        // SEEK_END 2 End-of-file
        switch (whence)
        {
        case SEEK_SET: ((IReader*)datasource)->seek((int)offset); break;
        case SEEK_CUR: ((IReader*)datasource)->advance((int)offset); break;
        case SEEK_END: ((IReader*)datasource)->seek((int)offset + ((IReader*)datasource)->length()); break;
        }
        return 0;
    },
    // close
    [](void* datasource) -> int {
        auto* file = static_cast<IReader*>(datasource);
        FS.r_close(file);
        return 0;
    },
    // tell
    [](void* datasource) -> long {
        const auto file = static_cast<IReader*>(datasource);
        return static_cast<long>(file->tell());
    },
};
}

namespace xrSound
{
Source::~Source() { unload(); }

bool Source::load(pcstr name)
{
    string_path fn, N;
    xr_strcpy(N, name);
#ifdef XR_PLATFORM_WINDOWS
    xr_strlwr(N);
#endif

    if (strext(N))
        *strext(N) = 0;

    fname = N;

    strconcat(fn, N, ".ogg");
    if (!FS.exist("$level$", fn))
        FS.update_path(fn, "$game_sounds$", fn);

#ifndef MASTER_GOLD
    if (!FS.exist(fn))
    {
        Msg("~ %s: Can't find sound '%s'", __FUNCTION__, name);
#ifdef _EDITOR
        FS.update_path(fn, "$game_sounds$", "$no_sound.ogg");
#endif
    }
#endif

    if (FS.exist(fn))
    {
        if (load_wave(fn))
            return true;
    }

    return false;
}

void Source::unload()
{
    m_time_total = 0.0f;
    m_bytes_total = 0;
}

OggVorbis_File* Source::open() const
{
    const auto file = FS.r_open(pname.c_str());
    R_ASSERT3(file && file->length(), "Can't open wave file:", pname.c_str());

    const auto ovf = xr_new<OggVorbis_File>();
    ov_open_callbacks(file, ovf, nullptr, 0, g_ov_callbacks);

    return ovf;
}

void Source::close(OggVorbis_File* ovf) const
{
    ov_clear(ovf);
    xr_delete(ovf);
}

void Source::decompress(void* dest, u32 byte_offset, u32 size, OggVorbis_File* ovf) const
{
    ZoneScoped;

    // seek
    const auto sample_offset = ogg_int64_t(byte_offset / m_data_info.block_align);
    const u32 cur_pos = u32(ov_pcm_tell(ovf));
    if (cur_pos != sample_offset)
        ov_pcm_seek(ovf, sample_offset);

    // decompress
    if (m_data_info.format == SoundFormat::Float32)
        decompress(ovf, static_cast<float*>(dest), size);
    else
        decompress(ovf, static_cast<char*>(dest), size);
}

bool Source::load_wave(pcstr pName)
{
    ZoneScoped;

    pname = pName;

    // Load file into memory and parse WAV-format
    OggVorbis_File ovf;
    {
        IReader* wave = FS.r_open(pname.c_str());
        R_ASSERT3(wave && wave->length(), "Can't open wave file:", pname.c_str());
        ov_open_callbacks(wave, &ovf, nullptr, 0, g_ov_callbacks);
    }

    const vorbis_info* ovi = ov_info(&ovf, -1);

    // verify
    R_ASSERT3_CURE(ovi, "Invalid source info:", pName,
    {
        ov_clear(&ovf);
        return false;
    });

    m_data_info = {};

    m_data_info.samples_per_sec = ovi->rate;
    m_data_info.channels = u16(ovi->channels);

    if (SoundRenderCore->supports_float_pcm)
    {
        m_data_info.format = SoundFormat::Float32;
        m_data_info.bits_per_sample = 32;
    }
    else
    {
        m_data_info.format = SoundFormat::PCM;
        m_data_info.bits_per_sample = 16;
    }

    m_data_info.block_align = m_data_info.bits_per_sample / 8 * m_data_info.channels;
    m_data_info.avg_bytes_per_sec = m_data_info.samples_per_sec * m_data_info.block_align;
    m_data_info.bytes_per_buffer = sdef_target_block * m_data_info.avg_bytes_per_sec / 1000;

    const s64 pcm_total = ov_pcm_total(&ovf, -1);
    m_bytes_total = u32(pcm_total * m_data_info.block_align);
    m_time_total = m_bytes_total / float(m_data_info.avg_bytes_per_sec);

    m_info = {};

    const vorbis_comment* ovm = ov_comment(&ovf, -1);
    if (ovm->comments)
    {
        IReader reader(ovm->user_comments[0], ovm->comment_lengths[0]);
        const u32 vers = reader.r_u32();
        if (vers == 0x0001)
        {
            m_info.min_dist = reader.r_float();
            m_info.max_dist = reader.r_float();
            m_info.base_volume = 1.f;
            m_info.game_type = reader.r_u32();
            m_info.max_ai_dist = m_info.max_dist;
        }
        else if (vers == 0x0002)
        {
            m_info.min_dist = reader.r_float();
            m_info.max_dist = reader.r_float();
            m_info.base_volume = reader.r_float();
            m_info.game_type = reader.r_u32();
            m_info.max_ai_dist = m_info.max_dist;
        }
        else if (vers == OGG_COMMENT_VERSION)
        {
            m_info.min_dist = reader.r_float();
            m_info.max_dist = reader.r_float();
            m_info.base_volume = reader.r_float();
            m_info.game_type = reader.r_u32();
            m_info.max_ai_dist = reader.r_float();
        }
        else
        {
#ifndef MASTER_GOLD
            Log("! Invalid ogg-comment version, file: ", pName);
#endif
        }
    }
    else
    {
#ifndef MASTER_GOLD
        Log("! Missing ogg-comment, file: ", pName);
#endif
    }

    R_ASSERT3_CURE(m_info.max_ai_dist >= 0.1f && m_info.max_dist >= 0.1f, "Invalid max distance.", pName,
    {
        ov_clear(&ovf);
        return false;
    });

    ov_clear(&ovf);
    return true;
}

void Source::decompress(OggVorbis_File* ovf, char* _dest, u32 size) const
{
    long total_ret = 0;

    // Read loop
    while (total_ret < static_cast<long>(size))
    {
        const auto ret = ov_read(ovf, _dest + total_ret, size - total_ret, 0, 2, 1, nullptr);
        if (ret <= 0 && !ov_can_continue_read(ret))
            break;
        total_ret += ret;
    }
}

void Source::decompress(OggVorbis_File* ovf, float* _dest, u32 size) const
{
    s32 left = s32(size / m_data_info.block_align);
    while (left)
    {
        float** pcm;
        long samples = ov_read_float(ovf, &pcm, left, nullptr);

        if (samples <= 0 && !ov_can_continue_read(samples))
            break;

        if (samples > left)
            samples = left;

        for (long j = 0; j < samples; j++)
            for (long i = 0; i < m_data_info.channels; i++)
                *_dest++ = pcm[i][j];

        left -= samples;
    }
}
}
