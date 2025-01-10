#include "stdafx.h"

#include "SoundRender_CoreA.h"

XRSOUND_API u32 snd_device_id = u32(-1);

ISoundScene* DefaultSoundScene{};

void CSoundManager::CreateDevicesList()
{
    ZoneScoped;

    static bool noSound = strstr(Core.Params, "-nosound");

    SoundRenderCore = xr_new<CSoundRender_CoreA>(*this);

    if (!noSound)
        SoundRenderCore->_initialize_devices_list();

    if (!SoundRenderCore->bPresent)
        soundDevices.emplace_back(nullptr, -1);

    GEnv.Sound = SoundRenderCore;
}

void CSoundManager::Create()
{
    ZoneScoped;

    if (SoundRenderCore->bPresent)
    {
        env_load();
        SoundRenderCore->_initialize();
    }
}

void CSoundManager::Destroy()
{
    ZoneScoped;

    GEnv.Sound = nullptr;

    SoundRenderCore->_clear();
    xr_delete(SoundRenderCore);

    env_unload();

    for (auto& token : soundDevices)
    {
        pstr tokenName = const_cast<pstr>(token.name);
        xr_free(tokenName);
    }
    soundDevices.clear();
}

bool CSoundManager::IsSoundEnabled() const
{
    return SoundRenderCore && SoundRenderCore->bPresent;
}

void CSoundManager::env_load()
{
    string_path fn;
    if (FS.exist(fn, "$game_data$", SNDENV_FILENAME))
    {
        soundEnvironment = xr_new<SoundEnvironment_LIB>();
        soundEnvironment->Load(fn);
    }
}

void CSoundManager::env_unload()
{
    if (soundEnvironment)
        soundEnvironment->Unload();
    xr_delete(soundEnvironment);
}

SoundEnvironment_LIB* CSoundManager::get_env_library() const
{
    return soundEnvironment;
}

void CSoundManager::refresh_env_library()
{
    env_unload();
    env_load();
    SoundRenderCore->env_apply();
}
