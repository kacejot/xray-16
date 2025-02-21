#include <stdafx.h>
#include "SourceStream.h"

SourceStream::SourceStream(pcstr name) noexcept : CSoundRender_Source(name)
{  
}

void SourceStream::decompress(void* dest, u32 byte_offset, u32 size) const 
{

}
