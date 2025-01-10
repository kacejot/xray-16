#include "stdafx.h"

#include "Include/xrAPI/xrAPI.h"
#include "Common/LevelStructure.hpp"
#include "SoundRender_Core.h"
#include "SoundRender_Source.h"
#include "SoundRender_Emitter.h"

#include "xrEngine/Engine.h"
#include "xrEngine/GameFont.h"
#include "xrEngine/PerformanceAlert.hpp"
#include "xrCDB/Intersect.hpp"
#include "SoundRender_Target.h"

XRSOUND_API Flags32 psSoundFlags =
{
    ss_Hardware | ss_EFX
};

XRSOUND_API int psSoundTargets = 32;
XRSOUND_API float psSoundOcclusionScale = 0.5f;
XRSOUND_API float psSoundTimeFactor = 1.0f;
XRSOUND_API float psSoundCull = 0.01f;
XRSOUND_API float psSoundRolloff = 0.75f;
XRSOUND_API u32 psSoundModel = 0;
XRSOUND_API float psSoundVEffects = 1.0f;
XRSOUND_API float psSoundVFactor = 1.0f;

XRSOUND_API float psSoundVMusic = 1.f;
XRSOUND_API int psSoundCacheSizeMB = 32;

CSoundRender_Core* SoundRenderCore = nullptr;

CSoundRender_Core::CSoundRender_Core(CSoundManager& p)
    : Parent(p)
{
    bPresent = false;
    s_emitters_u = 0;
    e_current.set_identity();
    e_target.set_identity();
    bReady = false;
    isLocked = false;
    fTimer_Value = Timer.GetElapsed_sec();
    fTimer_Delta = 0.0f;
    fTimerPersistent_Value = TimerPersistent.GetElapsed_sec();
    fTimerPersistent_Delta = 0.0f;
}

void CSoundRender_Core::_initialize()
{
    Timer.Start();
    TimerPersistent.Start();

    bPresent = true;

    bReady = true;
}

void CSoundRender_Core::_clear()
{
    bReady = false;

    // remove sources
    for (auto& kv : s_sources)
    {
        xr_delete(kv.second);
    }
    s_sources.clear();
}

ISoundScene* CSoundRender_Core::create_scene()
{
    return m_scenes.emplace_back(xr_new<CSoundRender_Scene>());
}

void CSoundRender_Core::destroy_scene(ISoundScene*& sound_scene)
{
    m_scenes.erase(std::remove(m_scenes.begin(), m_scenes.end(), sound_scene), m_scenes.end());
    xr_delete(sound_scene);
}

void CSoundRender_Core::stop_emitters()
{
    for (const auto& scene : m_scenes)
        scene->stop_emitters();
}

int CSoundRender_Core::pause_emitters(bool pauseState)
{
    int cnt = 0;
    for (const auto& scene : m_scenes)
        cnt += scene->pause_emitters(pauseState);
    return cnt;
}

void CSoundRender_Core::_restart()
{
    env_apply();
}

CSound* CSoundRender_Core::create(pcstr fName, esound_type sound_type, int game_type)
{
    if (!bPresent)
        return nullptr;

    string_path fn;
    xr_strcpy(fn, fName);
    if (strext(fn))
        *strext(fn) = 0;

    Source* handle = i_create_source(fn);
    if (!handle)
        return nullptr;

    auto* snd = xr_new<CSound>(handle);

    snd->g_type = game_type;
    if (game_type == sg_SourceType)
        snd->g_type = snd->handle->game_type();

    snd->s_type = sound_type;

    snd->dwBytesTotal = snd->handle->bytes_total();
    snd->fTimeTotal = snd->handle->length_sec();

    return snd;
}

void CSoundRender_Core::attach_tail(CSound& snd, pcstr fName)
{
    if (!bPresent)
        return;
    string_path fn;
    xr_strcpy(fn, fName);
    if (strext(fn))
        *strext(fn) = 0;
    if (!snd.fn_attached[0].empty() && !snd.fn_attached[1].empty())
    {
#ifndef MASTER_GOLD
        Msg("! 2 file already in queue [%s][%s]", snd.fn_attached[0].c_str(), snd.fn_attached[1].c_str());
#endif
        return;
    }

    const u32 idx = snd.fn_attached[0].empty() ? 0 : 1;

    snd.fn_attached[idx] = fn;

    Source* s = i_create_source(fn);
    snd.dwBytesTotal += s->bytes_total();
    snd.fTimeTotal += s->length_sec();
    if (snd.feedback)
        ((CSoundRender_Emitter*)snd.feedback)->fTimeToStop += s->length_sec();

    i_destroy_source(s);
}

void CSoundRender_Core::destroy(CSound& S)
{
    if (auto* emitter = (CSoundRender_Emitter*)S.feedback)
    {
        emitter->stop(false);
        VERIFY(S.feedback == nullptr);
    }
    i_destroy_source((Source*)S.handle);
    S.handle = nullptr;
}

void CSoundRender_Core::env_apply()
{
    /*
    // Force all sounds to change their environment
    // (set their positions to signal changes in environment)
    for (u32 it = 0; it < s_emitters.size(); it++)
    {
        CSoundRender_Emitter* pEmitter = s_emitters[it];
        const CSound_params* pParams = pEmitter->get_params();
        pEmitter->set_position(pParams->position);
    }
    */
    bListenerMoved = true;
}

void CSoundRender_Core::update_listener(const Fvector& P, const Fvector& D, const Fvector& N, const Fvector& R, float dt)
{
    if (!Listener.position.similar(P))
    {
        Listener.position = P;
        bListenerMoved = true;
    }
    Listener.orientation[0] = D;
    Listener.orientation[1] = N;
    Listener.orientation[2] = R;

    if (!psSoundFlags.test(ss_EFX) || !m_effects)
        return;

    // Update effects
    if (bListenerMoved)
    {
        bListenerMoved = false;
        e_target = *(CSoundRender_Environment*)DefaultSoundScene->get_environment(P);
    }

    e_current.lerp(e_current, e_target, fTimer_Delta);

    m_effects->set_listener(e_current);
    m_effects->commit();
}

void CSoundRender_Core::refresh_sources()
{
    stop_emitters();

    for (const auto& kv : s_sources)
    {
        Source* s = kv.second;
        s->unload();
        s->load(s->file_name());
    }
}

Source* CSoundRender_Core::i_create_source(pcstr name)
{
    // Search
    string256 id;
    xr_strcpy(id, name);
    xr_strlwr(id);
    if (strext(id))
        *strext(id) = 0;

    {
        ScopeLock scope(&s_sources_lock);
        const auto it = s_sources.find(id);
        if (it != s_sources.end())
        {
            return it->second;
        }
    }

    // Load a _new one
    Source source;
    if (source.load(id))
    {
        ScopeLock scope(&s_sources_lock);
        Source* S = xr_new<Source>(std::move(source));
        s_sources.emplace(id, S);
        return S;
    }

    return nullptr;
}

void CSoundRender_Core::i_destroy_source(Source* S)
{
    // No actual destroy at all
}

void CSoundRender_Core::i_start(CSoundRender_Emitter* E) const
{
    R_ASSERT1_CURE(E, { return; });

    // Search lowest-priority target
    float Ptarget = flt_max;
    CSoundRender_Target* T = nullptr;
    for (const auto Ttest : s_targets)
    {
        if (Ttest->get_priority() < Ptarget)
        {
            T = Ttest;
            Ptarget = Ttest->get_priority();
        }
    }

    // Stop currently playing
    if (T->get_emitter())
        T->get_emitter()->cancel();

    // Associate
    E->target = T;
    E->target->start(E);
}

bool CSoundRender_Core::i_allow_play(const CSoundRender_Emitter* E)
{
    // Search available target
    const float Ptest = E->priority();
    return std::any_of(s_targets.begin(), s_targets.end(),
        [Ptest](const CSoundRender_Target* target) { return target->get_priority() < Ptest; });
}

void CSoundRender_Core::update(const Fvector& P, const Fvector& D, const Fvector& N, const Fvector& R)
{
    ZoneScoped;

    if (0 == bReady)
        return;
    Stats.Update.Begin();
    isLocked = true;

    Timer.time_factor(psSoundTimeFactor); //--#SM+#--
    {
        const float new_tm = Timer.GetElapsed_sec();
        fTimer_Delta = new_tm - fTimer_Value;
        fTimer_Value = new_tm;

        const float new_tm_p = TimerPersistent.GetElapsed_sec();
        fTimerPersistent_Delta = new_tm_p - fTimerPersistent_Value;
        fTimerPersistent_Value = new_tm_p;
    }
    s_emitters_u++;

    const auto update_emitter = [this](CSoundRender_Emitter* emitter) {
        const bool ignore = emitter->bIgnoringTimeFactor;
        const float time = ignore ? fTimerPersistent_Value : fTimer_Value;
        const float delta = ignore ? fTimerPersistent_Delta : fTimer_Delta;
        emitter->update(time, delta);
        emitter->marker = s_emitters_u;
    };

    // Firstly update emitters, which are now being rendered
    for (CSoundRender_Target* T : s_targets)
    {
        if (CSoundRender_Emitter* E = T->get_emitter())
        {
            update_emitter(E);
        }
    }

    // Update emitters
    for (CSoundRender_Scene* scene : m_scenes)
    {
        auto& emitters = scene->get_emitters();
        for (u32 it = 0; it < emitters.size(); it++)
        {
            CSoundRender_Emitter* pEmitter = emitters[it];
            if (pEmitter->marker != s_emitters_u)
            {
                update_emitter(pEmitter);
            }
            if (!pEmitter->isPlaying())
            {
                // Stopped
                xr_delete(pEmitter);
                emitters.erase(emitters.begin() + it);
                it--;
            }
        }
    }

    // update listener
    update_listener(P, D, N, R, fTimer_Delta);

    // Events
    for (CSoundRender_Scene* scene : m_scenes)
        scene->update();

    isLocked = false;
    Stats.Update.End();
}

void CSoundRender_Core::render()
{
    ZoneScoped;

    isLocked = true;
    Stats.Render.Begin();

    for (CSoundRender_Target* T : s_targets)
    {
        if (CSoundRender_Emitter* emitter = T->get_emitter())
        {
            emitter->render();
        }
    }

    Stats.Render.End();
    isLocked = false;
}

void CSoundRender_Core::statistic(CSound_stats* dest, CSound_stats_ext* ext)
{
    if (dest)
    {
        dest->_rendered = 0;
        dest->_simulated = 0;
        dest->_events = 0;

        for (auto T : s_targets)
        {
            if (T->get_emitter() && T->get_Rendering())
                dest->_rendered++;
        }

        for (CSoundRender_Scene* scene : m_scenes)
        {
            dest->_simulated += scene->get_emitters().size();
            dest->_events += scene->get_prev_events_count();
        }
    }
    if (ext)
    {
        for (CSoundRender_Scene* scene : m_scenes)
        {
            auto& emitters = scene->get_emitters();
            for (const auto emitter : emitters)
            {
                CSound_stats_ext::SItem item;
                item._3D = !emitter->b2D;
                item._rendered = !!emitter->target;
                item.params = emitter->p_source;
                item.volume = emitter->smooth_volume;
                if (emitter->owner_data)
                {
                    item.name = emitter->source()->file_name();
                    item.game_object = emitter->owner_data->g_object;
                    item.game_type = emitter->owner_data->g_type;
                    item.type = emitter->owner_data->s_type;
                }
                else
                {
                    item.game_object = nullptr;
                    item.game_type = 0;
                    item.type = st_Effect;
                }
                ext->append(item);
            }
        }
    }
}

void CSoundRender_Core::DumpStatistics(IGameFont& font, IPerformanceAlert* alert)
{
    Stats.FrameEnd();
    CSound_stats sndStat;
    statistic(&sndStat, nullptr);
    font.OutNext("*** SOUND:    %2.2fms", Stats.Update.result);
    font.OutNext("    RENDER:   %2.2fms", Stats.Render.result);
    font.OutNext("Rendered:     %d", sndStat._rendered);
    font.OutNext("Simulated:    %d", sndStat._simulated);
    font.OutNext("Events:       %d", sndStat._events);
    Stats.FrameStart();
}
