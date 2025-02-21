#pragma once
#include "SoundRender_Source.h"

class BufferSource : public CSoundRender_Source
{
public:
    BufferSource();
    void decompress(void* dest, u32 byte_offset, u32 size) const override;

private:
    xr_vector<u8> m_buffer;
};
