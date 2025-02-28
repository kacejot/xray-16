#include "stdafx.h"
#include "SoundRender_Source.h"

CSoundRender_Source::CSoundRender_Source(OggVorbisFile&& ovf) noexcept : m_ovf(std::move(ovf))
{
}

BufferSource::BufferSource(OggVorbisFile&& ovf) : CSoundRender_Source(std::move(ovf))
{
    m_ovf.decompress(m_buffer.data(), 0, m_ovf.info().bytes_total);
}

void BufferSource::decompress(void* dest, u32 byte_offset, u32 size)
{
    memcpy(dest, m_buffer.data() + byte_offset, size);
}

SourceStream::SourceStream(OggVorbisFile&& ovf) noexcept : CSoundRender_Source(std::move(ovf))
{
}

void SourceStream::decompress(void* dest, u32 byte_offset, u32 size)
{
    m_ovf.decompress(dest, byte_offset, size);
}




