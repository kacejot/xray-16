#pragma once

struct OggVorbis_File;

enum class SoundFormat
{
    Unknown,
    PCM,
    Float32,
};

struct SoundDataInfo
{
    SoundFormat format{};
    u16         channels{};       // number of channels (i.e. mono, stereo...)
    u32         samplesPerSec{};  // sample rate
    u32         avgBytesPerSec{}; // for buffer estimation
    u16         blockAlign{};     // block size of data
    u16         bitsPerSample{};  // number of bits per sample of mono data
    u32         bytesPerBuffer{}; // target buffer size
};

struct SoundSourceInfo
{
    float baseVolume{ 1.0f };
    float minDist   { 1.0f };
    float maxDist   { 300.0f };
    float maxAIDist { 300.0f };
    u32   gameType  {};
};

class OggVorbisFileGuard
{
public:
    OggVorbisFileGuard() = default;
    ~OggVorbisFileGuard();
    OggVorbisFileGuard(const OggVorbisFileGuard&) = delete;
    OggVorbisFileGuard& operator=(const OggVorbisFileGuard&) = delete;

    OggVorbis_File* create();
    OggVorbis_File* get() const;
    bool has_value() const;
    void clear();

private:
    OggVorbis_File* m_ovf = nullptr;
};

class XRSOUND_API CSoundRender_Source final : public CSound_source
{
public:
    CSoundRender_Source(pcstr name) noexcept;
    ~CSoundRender_Source() override = default;

    CSoundRender_Source(const CSoundRender_Source&) = delete;
    CSoundRender_Source(CSoundRender_Source&&) noexcept = default;

    CSoundRender_Source& operator=(const CSoundRender_Source&) = delete;
    CSoundRender_Source& operator=(CSoundRender_Source&&) noexcept = default;

    const OggVorbisFileGuard& ovf() const;
    void decompress(void* dest, u32 byte_offset, u32 size, OggVorbis_File* ovf) const;

    [[nodiscard]] const auto& data_info() const { return m_data_info; }
    [[nodiscard]] const auto&      info() const { return m_info; }

    [[nodiscard]] pcstr file_name() const override { return m_filename.c_str(); }

    [[nodiscard]] float length_sec() const override { return m_time_total; }
    [[nodiscard]] u32 bytes_total() const override { return m_bytes_total; }

    [[nodiscard]] u16 channels_num() const override { return data_info().channels; }
    [[nodiscard]] u32 game_type() const override { return info().gameType; }

private:
    void i_decompress(OggVorbis_File* ovf, char* dest, u32 size) const;
    void i_decompress(OggVorbis_File* ovf, float* dest, u32 size) const;
    void load_wave(pcstr name);

private:
    SoundDataInfo m_data_info{};
    SoundSourceInfo m_info{};

    OggVorbisFileGuard m_ovf;
    shared_str m_filename;

    u32 m_bytes_total{};
    float m_time_total{};
};
