#include "stdafx.h"

#include "SoundRender_Core.h"
#include "SoundRender_Source.h"

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
        const auto it = s_sources.find(id);
        if (it != s_sources.end())
        {
            return it->second.get();
        }
    }

    // Load a _new one
    auto source = xr_make_unique<CSoundRender_Source>(id);
    if (source->ovf().has_value())
    {
        ScopeLock scope(&s_sources_lock);
        auto source_ptr = source.get();
        s_sources.emplace(id, std::move(source));
        return source_ptr;
    }

    return nullptr;
}

void CSoundRender_Core::i_destroy_source(CSoundRender_Source* S)
{
    // No actual destroy at all
}
