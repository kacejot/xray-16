#include "stdafx.h"

#include "SoundRender_Core.h"

CSoundRender_Source* CSoundRender_Core::i_create_source(pcstr name)
{
    // Search
    string256 id;
    xr_strcpy(id, name);
    xr_strlwr(id);
    if (strext(id))
        * strext(id) = 0;

    {
        ScopeLock scope(&s_sources_lock);
        auto source = m_source_cache.get(id);
        if (nullptr != source)
        {
            return source;
        }
    }

    return nullptr;
}

void CSoundRender_Core::i_destroy_source(CSoundRender_Source* S)
{
    // No actual destroy at all
}
