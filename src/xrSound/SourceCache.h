#pragma once
#include "xrCommon/xr_unordered_map.h"
#include "xrCommon/xr_deque.h"
#include "SoundRender.h"

class CSoundRender_Source;

class SourceCache
{
public:
    explicit SourceCache(u32 max_size = sdef_max_cache_size);

    SourceCache(const SourceCache&) = delete;
    SourceCache& operator=(const SourceCache&) = delete;

    SourceCache(SourceCache&&) noexcept = default;
    SourceCache& operator=(SourceCache&&) noexcept = default;

    void clear() { m_sources.clear(); m_cache_order.clear(); }

    [[nodiscard]] CSoundRender_Source* get(pcstr name);

private:
    xr_unordered_map<xr_string, xr_unique_ptr<CSoundRender_Source>> m_sources;
    xr_deque<xr_string> m_cache_order;
    u32 m_max_size;
};
