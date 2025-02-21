#pragma once

#include <vorbis/vorbisfile.h>

enum class SoundFormat
{
    Unknown,
    PCM,
    Float32,
};

struct WaveInfo
{
    SoundFormat format{};
    u32 bytes_per_buffer{};   // target buffer size
    u32 samples_per_sec{};    // sample rate
    u32 avg_bytes_per_sec{};  // for buffer estimation
    u32 game_type{};
    u32 bytes_total{};
    float time_total{};
    float base_volume{1.0f};
    float min_dist{1.0f};
    float max_dist{300.0f};
    float max_ai_dist{300.0f};
    u16 channels{};            // number of channels (i.e. mono, stereo...)
    u16 block_align{};         // block size of data
    u16 bits_per_sample{};     // number of bits per sample of mono data
};

class OggVorbisFile
{
public:
    OggVorbisFile() noexcept = default;
    ~OggVorbisFile();

    // forbid copy
    // allow move

    bool load(pcstr name);
    void decompress(void* dest, u32 byte_offset, u32 size);
    [[nodiscard]] const auto& info() const { return m_info; }

private:
    void clear();
    void decompress_char(char* dest, u32 size);
    void decompress_float(float* dest, u32 size);

private:
    WaveInfo m_info{};
    shared_str m_filename;
    OggVorbis_File m_ovf;
};
