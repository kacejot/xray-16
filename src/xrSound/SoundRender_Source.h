#pragma once

#include "OggVorbisFile.h"

class XRSOUND_API CSoundRender_Source : public CSound_source
{
public:
    CSoundRender_Source(OggVorbisFile&&) noexcept;
    ~CSoundRender_Source() override = default;

    CSoundRender_Source(const CSoundRender_Source&) = delete;
    CSoundRender_Source(CSoundRender_Source&&) noexcept = default;

    CSoundRender_Source& operator=(const CSoundRender_Source&) = delete;
    CSoundRender_Source& operator=(CSoundRender_Source&&) noexcept = default;

    virtual void decompress(void* dest, u32 byte_offset, u32 size) = 0;

    [[nodiscard]] bool is_valid() const { return m_is_valid; }
    [[nodiscard]] const auto& info() const { return m_ovf.info(); }
    [[nodiscard]] pcstr file_name() const override { return m_ovf.file_name(); }
    [[nodiscard]] float length_sec() const override { return m_ovf.info().time_total; }
    [[nodiscard]] u32 bytes_total() const override { return m_ovf.info().bytes_total; }
    [[nodiscard]] u16 channels_num() const override { return m_ovf.info().channels; }
    [[nodiscard]] u32 game_type() const override { return m_ovf.info().game_type; }

protected:
    OggVorbisFile m_ovf;
    bool m_is_valid = false;
};

class BufferSource : public CSoundRender_Source
{
public:
    BufferSource(OggVorbisFile&&);
    void decompress(void* dest, u32 byte_offset, u32 size) override;

private:
    xr_vector<u8> m_buffer;
};

class SourceStream : public CSoundRender_Source
{
public:
    SourceStream(OggVorbisFile&&) noexcept;
    void decompress(void* dest, u32 byte_offset, u32 size) override;
};
