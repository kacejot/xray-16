#pragma once
#include "SoundRender_Source.h"

class SourceStream : public CSoundRender_Source
{
public:
    SourceStream(pcstr name) noexcept;
    void decompress(void* dest, u32 byte_offset, u32 size) const override;
};
