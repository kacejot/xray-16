#include "stdafx.h"
#include "SourceCache.h"
#include "SoundRender_Source.h"
#include "OggVorbisFile.h"

namespace
{
xr_unique_ptr<CSoundRender_Source> create_source(pcstr name)
{
    auto [is_valid, ovf] = OggVorbisFile::from_file(name);
    if (!is_valid)
        return nullptr;

    if (ovf.info().bytes_total <= sdef_max_cached_file_size)
        return xr_make_unique<BufferSource>(name);
    else
        return xr_make_unique<SourceStream>(name);
}
}

SourceCache::SourceCache(u32 max_size) : m_max_size(max_size) 
{
}

CSoundRender_Source* SourceCache::get(pcstr name)
{
    const auto it = m_sources.find(name);
    if (it != m_sources.end())
    {
        m_cache_order.erase(std::find(m_cache_order.begin(), m_cache_order.end(), name)); // both find and erase are O(n)
        m_cache_order.push_front(name);
        return it->second.get();
    }

    auto source = create_source(name);
    if (!source)
        return nullptr;

   m_sources[name] = std::move(source);
   m_cache_order.push_front(name);

    if (m_sources.size() > m_max_size)
    {
        m_sources.erase(m_cache_order.back());
        m_cache_order.pop_back();
    }

    return m_sources[name].get();
}
