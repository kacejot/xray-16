#pragma once

#include "OggVorbisFile.h"

class XRSOUND_API CSoundRender_Source : public CSound_source
{
public:
    CSoundRender_Source(pcstr name) noexcept;
    ~CSoundRender_Source() override = default;

    CSoundRender_Source(const CSoundRender_Source&) = delete;
    CSoundRender_Source(CSoundRender_Source&&) noexcept = default;

    CSoundRender_Source& operator=(const CSoundRender_Source&) = delete;
    CSoundRender_Source& operator=(CSoundRender_Source&&) noexcept = default;

    virtual void decompress(void* dest, u32 byte_offset, u32 size) const = 0;

    [[nodiscard]] bool is_valid() const { return m_is_valid; }
    [[nodiscard]] const auto& info() const { return m_ovf.info(); }
    [[nodiscard]] pcstr file_name() const override { return m_filename.c_str(); }
    [[nodiscard]] float length_sec() const override { return m_ovf.info().time_total; }
    [[nodiscard]] u32 bytes_total() const override { return m_ovf.info().bytes_total; }
    [[nodiscard]] u16 channels_num() const override { return m_ovf.info().channels; }
    [[nodiscard]] u32 game_type() const override { return m_ovf.info().game_type; }

protected:
    OggVorbisFile m_ovf;
    shared_str m_filename;
    bool m_is_valid = false;
};
