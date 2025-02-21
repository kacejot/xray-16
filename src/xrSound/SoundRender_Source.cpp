#include "stdafx.h"
#include "SoundRender_Source.h"

CSoundRender_Source::CSoundRender_Source(pcstr filename) noexcept
{
    string_path fn, N;
    xr_strcpy(N, filename);
#ifdef XR_PLATFORM_WINDOWS
    xr_strlwr(N);
#endif

    if (strext(N))
        *strext(N) = 0;

    m_filename = N;

    strconcat(fn, N, ".ogg");
    if (!FS.exist("$level$", fn))
        FS.update_path(fn, "$game_sounds$", fn);

#ifndef MASTER_GOLD
    if (!FS.exist(fn))
    {
        Msg("~ %s: Can't find sound '%s'", __FUNCTION__, filename);
#ifdef _EDITOR
        FS.update_path(fn, "$game_sounds$", "$no_sound.ogg");
#endif
    }
#endif

    if (FS.exist(fn))
    {
        m_is_valid = m_ovf.load(fn);
    }
}





