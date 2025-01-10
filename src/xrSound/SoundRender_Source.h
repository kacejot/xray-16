#pragma once

struct OggVorbis_File;

namespace xrSound
{
enum class SoundFormat
{
    Unknown,
    PCM,
    Float32,
};

struct SoundDataInfo
{
    SoundFormat format{};
    u16 channels{}; // number of channels (i.e. mono, stereo...)
    u32 samples_per_sec{}; // sample rate
    u32 avg_bytes_per_sec{}; // for buffer estimation
    u16 block_align{}; // block size of data
    u16 bits_per_sample{}; // number of bits per sample of mono data
    u32 bytes_per_buffer{}; // target buffer size
};

struct SoundSourceInfo
{
    float base_volume{1.0f};
    float min_dist{1.0f};
    float max_dist{300.0f};
    float max_ai_dist{300.0f};
    u32 game_type{};
};

class XRSOUND_API Source final : public CSound_source
{
public:
    ~Source() override;

    bool load(pcstr name);
    void unload();

    OggVorbis_File* open() const;
    void close(OggVorbis_File* ovf) const;

    void decompress(void* dest, u32 byte_offset, u32 size, OggVorbis_File* ovf) const;

    [[nodiscard]] const auto& data_info() const { return m_data_info; }
    [[nodiscard]] const auto& info() const { return m_info; }

    [[nodiscard]] pcstr file_name() const override { return fname.c_str(); }

    [[nodiscard]] float length_sec() const override { return m_time_total; }
    [[nodiscard]] u32 bytes_total() const override { return m_bytes_total; }

    [[nodiscard]] u16 channels_num() const override { return data_info().channels; }
    [[nodiscard]] u32 game_type() const override { return info().game_type; }

private:
    void decompress(OggVorbis_File* ovf, char* dest, u32 size) const;
    void decompress(OggVorbis_File* ovf, float* dest, u32 size) const;
    bool load_wave(pcstr name);

private:
    shared_str pname;
    shared_str fname;

    float m_time_total{};
    u32 m_bytes_total{};

    SoundDataInfo m_data_info{};
    SoundSourceInfo m_info{};
};
}
