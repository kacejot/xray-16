#include "stdafx.h"

#include "SoundRender_Emitter.h"
#include "SoundRender_Core.h"
#include "SoundRender_Scene.h"
#include "SoundRender_Source.h"

#include "xrCore/Threading/TaskManager.hpp"

extern u32 psSoundModel;
extern float psSoundVEffects;

void CSoundRender_Emitter::set_position(const Fvector& pos)
{
    if (source()->channels_num() == 1)
        p_source.position = pos;
    else
        p_source.position.set(0, 0, 0);

    bMoved = true;
}

void CSoundRender_Emitter::set_frequency(float scale)
{
    VERIFY(_valid(scale));
    if (_valid(scale))
        p_source.freq = scale;
}

// Перемотка звука на заданную секунду [rewind snd to target time] --#SM+#--
void CSoundRender_Emitter::set_time(float t)
{
    VERIFY2(get_length_sec() >= t, "set_time: time is bigger than length of sound");
    clamp(t, 0.0f, get_length_sec());
    fTimeToRewind = t;
}

CSoundRender_Emitter::CSoundRender_Emitter(CSoundRender_Scene* s)
    : scene(s),
      priority_scale(1.f),
      smooth_volume(1.f),
      occluder_volume(1.f),
      fade_volume(1.f),
      m_current_state(stStopped),
      bMoved(true),
      marker(0xabababab) {}

CSoundRender_Emitter::~CSoundRender_Emitter()
{
    // try to release dependencies, events, for example
    Event_ReleaseOwner();
    wait_prefill();
}

//////////////////////////////////////////////////////////////////////
void CSoundRender_Emitter::Event_ReleaseOwner()
{
    if (!owner_data)
        return;

    auto& events = scene->get_events();

    for (u32 it = 0; it < events.size(); it++)
    {
        if (owner_data == events[it].first)
        {
            events.erase(events.begin() + it);
            it--;
        }
    }
}

void CSoundRender_Emitter::Event_Propagade()
{
    fTimeToPropagade += ::Random.randF(s_f_def_event_pulse - 0.030f, s_f_def_event_pulse + 0.030f);
    if (!owner_data)
        return;
    if (!owner_data->g_type)
        return;
    if (!owner_data->g_object)
        return;
    if (!scene->get_events_handler())
        return;

    VERIFY(_valid(p_source.volume));
    // Calculate range
    const float clip = p_source.max_ai_distance * p_source.volume;
    const float range = std::min(p_source.max_ai_distance, clip);
    if (range < 0.1f)
        return;

    // Inform objects
    scene->get_events().emplace_back(owner_data, range);
}

void CSoundRender_Emitter::switch_to_2D()
{
    b2D = true;
    set_priority(100.f);
}

void CSoundRender_Emitter::switch_to_3D()
{
    b2D = false;
}

u32 CSoundRender_Emitter::play_time()
{
    if (m_current_state == stPlaying || m_current_state == stPlayingLooped || m_current_state == stSimulating ||
        m_current_state == stSimulatingLooped)
        return iFloor((SoundRenderCore->fTimer_Value - fTimeStarted) * 1000.0f);
    return 0;
}

void CSoundRender_Emitter::set_cursor(u32 p)
{
    m_stream_cursor = p;

    if (owner_data._get() && owner_data->fn_attached[0].size())
    {
        u32 bt = ((Source*)owner_data->handle)->bytes_total();
        if (m_stream_cursor >= m_cur_handle_cursor + bt)
        {
            SoundRenderCore->i_destroy_source((Source*)owner_data->handle);
            owner_data->handle = SoundRenderCore->i_create_source(owner_data->fn_attached[0].c_str());
            owner_data->fn_attached[0] = owner_data->fn_attached[1];
            owner_data->fn_attached[1] = "";
            m_cur_handle_cursor = get_cursor(true);
        }
    }
}

u32 CSoundRender_Emitter::get_cursor(bool b_absolute) const
{
    if (b_absolute)
        return m_stream_cursor;
    VERIFY(m_stream_cursor - m_cur_handle_cursor >= 0);
    return m_stream_cursor - m_cur_handle_cursor;
}

void CSoundRender_Emitter::move_cursor(int offset)
{
    set_cursor(get_cursor(true) + offset);
}

void CSoundRender_Emitter::fill_data(void* dest, u32 offset, u32 size) const
{
    source()->decompress(dest, offset, size, ovf);
}

void CSoundRender_Emitter::fill_block(void* ptr, u32 size)
{
    ZoneScoped;

    // Msg			("stream: %10s - [%X]:%d, p=%d, t=%d",*source->fname,ptr,size,position,source->dwBytesTotal);
    u8* dest = (u8*)(ptr);
    const u32 dwBytesTotal = get_bytes_total();

    if ((get_cursor(true) + size) > dwBytesTotal)
    {
        // We are reaching the end of data, what to do?
        switch (m_current_state)
        {
        case stPlaying:
        { // Fill as much data as we can, zeroing remainder
            if (get_cursor(true) >= dwBytesTotal)
            {
                // ??? We requested the block after remainder - just zero
                memset(dest, 0, size);
            }
            else
            {
                // Calculate remainder
                const u32 sz_data = dwBytesTotal - get_cursor(true);
                const u32 sz_zero = (get_cursor(true) + size) - dwBytesTotal;
                VERIFY(size == (sz_data + sz_zero));
                fill_data(dest, get_cursor(false), sz_data);
                memset(dest + sz_data, 0, sz_zero);
            }
            move_cursor(size);
        }
        break;
        case stPlayingLooped:
        {
            u32 hw_position = 0;
            do
            {
                u32 sz_data = dwBytesTotal - get_cursor(true);
                const u32 sz_write = std::min(size - hw_position, sz_data);
                fill_data(dest + hw_position, get_cursor(true), sz_write);
                hw_position += sz_write;
                move_cursor(sz_write);
                set_cursor(get_cursor(true) % dwBytesTotal);
            } while (0 != (size - hw_position));
        }
        break;
        default: FATAL("SOUND: Invalid emitter state"); break;
        }
    }
    else
    {
        const u32 bt_handle = ((Source*)owner_data->handle)->bytes_total();
        if (get_cursor(true) + size > m_cur_handle_cursor + bt_handle)
        {
            R_ASSERT(owner_data->fn_attached[0].size());

            u32 rem = 0;
            if ((m_cur_handle_cursor + bt_handle) > get_cursor(true))
            {
                rem = (m_cur_handle_cursor + bt_handle) - get_cursor(true);

#ifdef DEBUG
                Msg("reminder from prev source %d", rem);
#endif // #ifdef DEBUG
                fill_data(dest, get_cursor(false), rem);
                move_cursor(rem);
            }
#ifdef DEBUG
            Msg("recurce from next source %d", size - rem);
#endif // #ifdef DEBUG
            fill_block(dest + rem, size - rem);
        }
        else
        {
            // Everything OK, just stream
            fill_data(dest, get_cursor(false), size);
            move_cursor(size);
        }
    }
}

std::pair<u8*, size_t> CSoundRender_Emitter::obtain_block()
{
    wait_prefill();
    const std::pair result = { temp_buf[current_block].data(), temp_buf[current_block].size() };
    ++current_block;
    if (current_block >= sdef_target_count_prefill)
        current_block = 0;
    --filled_blocks;
    return std::move(result);
}

void CSoundRender_Emitter::fill_all_blocks()
{
    current_block = 0;
    for (size_t i = 0; i < sdef_target_count_prefill; ++i)
        fill_block(temp_buf[i].data(), temp_buf[i].size());
    filled_blocks = sdef_target_count_prefill;
}

void CSoundRender_Emitter::dispatch_prefill()
{
    wait_prefill();
    if (filled_blocks >= sdef_target_count_prefill)
        return;

    const auto task = &TaskScheduler->AddTask([this]
    {
        size_t next_block_to_fill = (current_block + filled_blocks) % sdef_target_count_prefill;

        while (filled_blocks < sdef_target_count_prefill)
        {
            auto& block = temp_buf[next_block_to_fill];

            fill_block(block.data(), block.size());

            next_block_to_fill = (next_block_to_fill + 1) % sdef_target_count_prefill;
            filled_blocks++;
        }

        prefill_task.store(nullptr, std::memory_order_release);
    });

    prefill_task.store(task, std::memory_order_release);
}

void CSoundRender_Emitter::wait_prefill() const
{
    if (const auto task = prefill_task.load(std::memory_order_acquire))
        TaskScheduler->Wait(*task);
}

Source* CSoundRender_Emitter::source() const
{
    return dynamic_cast<Source*>(owner_data->handle);
}

u32 CSoundRender_Emitter::get_bytes_total() const
{
    return owner_data->dwBytesTotal;
}

float CSoundRender_Emitter::get_length_sec() const
{
    return owner_data->fTimeTotal;
}

XRSOUND_API extern float psSoundCull;

inline u32 calc_cursor(const float& fTimeStarted, float& fTime, const float& fTimeTotal, const float& fFreq,
    const SoundDataInfo& info) //--#SM+#--
{
    if (fTime < fTimeStarted)
        fTime = fTimeStarted; // Андрюха посоветовал, ассерт что ниже вылетел из за паузы как то хитро
    R_ASSERT((fTime - fTimeStarted) >= 0.0f);
    while ((fTime - fTimeStarted) > fTimeTotal / fFreq) // looped
    {
        fTime -= fTimeTotal / fFreq;
    }
    const u32 curr_sample_num = iFloor((fTime - fTimeStarted) * fFreq * info.samples_per_sec);
    return curr_sample_num * (info.bits_per_sample / 8) * info.channels;
}

void CSoundRender_Emitter::update(float fTime, float dt)
{
    ZoneScoped;

    VERIFY2(!!(owner_data) || (!(owner_data) && (m_current_state == stStopped)), "owner");
    VERIFY2(owner_data ? *(int*)(&owner_data->feedback) : 1, "owner");

    if (bRewind)
    {
        wait_prefill();

        const float time =
            bIgnoringTimeFactor ? SoundRenderCore->TimerPersistent.GetElapsed_sec() : SoundRenderCore->Timer.GetElapsed_sec();
        const float diff = time - fTimeStarted;
        fTimeStarted += diff;
        fTimeToStop += diff;
        fTimeToPropagade = time;

        set_cursor(0);
        if (target)
        {
            fill_all_blocks();
            target->rewind();
            dispatch_prefill();
        }
        bRewind = FALSE;
    }

    switch (m_current_state)
    {
    case stStopped: break;
    case stStartingDelayed:
        if (iPaused)
            break;
        starting_delay -= dt;
        if (starting_delay <= 0)
            m_current_state = stStarting;
        break;
    case stStarting:
        if (iPaused)
            break;
        fTimeStarted = fTime;
        fTimeToStop = fTime + (get_length_sec() / p_source.freq); //--#SM+#--
        fTimeToPropagade = fTime;
        fade_volume = 1.f;
        occluder_volume = scene->get_occlusion(p_source.position, .2f, occluder);
        smooth_volume = p_source.base_volume * p_source.volume *
            (owner_data->s_type == st_Effect ? psSoundVEffects * psSoundVFactor : psSoundVMusic) *
            (b2D ? 1.f : occluder_volume);
        e_current = e_target = *(CSoundRender_Environment*)scene->get_environment(p_source.position);
        if (update_culling(dt))
        {
            m_current_state = stPlaying;
            set_cursor(0);
            SoundRenderCore->i_start(this);
            dispatch_prefill();
        }
        else
            m_current_state = stSimulating;
        break;
    case stStartingLoopedDelayed:
        if (iPaused)
            break;
        starting_delay -= dt;
        if (starting_delay <= 0)
            m_current_state = stStartingLooped;
        break;
    case stStartingLooped:
        if (iPaused)
            break;
        fTimeStarted = fTime;
        fTimeToStop = TIME_TO_STOP_INFINITE;
        fTimeToPropagade = fTime;
        fade_volume = 1.f;
        occluder_volume = scene->get_occlusion(p_source.position, .2f, occluder);
        smooth_volume = p_source.base_volume * p_source.volume *
            (owner_data->s_type == st_Effect ? psSoundVEffects * psSoundVFactor : psSoundVMusic) *
            (b2D ? 1.f : occluder_volume);
        e_current = e_target = *(CSoundRender_Environment*)scene->get_environment(p_source.position);
        if (update_culling(dt))
        {
            m_current_state = stPlayingLooped;
            set_cursor(0);
            SoundRenderCore->i_start(this);
            dispatch_prefill();
        }
        else
            m_current_state = stSimulatingLooped;
        break;
    case stPlaying:
        if (iPaused)
        {
            stop_target();
            m_current_state = stSimulating;
            fTimeStarted += dt;
            fTimeToStop += dt;
            fTimeToPropagade += dt;
            break;
        }
        if (fTime >= fTimeToStop)
        {
            // STOP
            stop_target();
            m_current_state = stStopped;
        }
        else
        {
            if (!update_culling(dt))
            {
                // switch to: SIMULATE
                stop_target();
                m_current_state = stSimulating;
            }
            else
            {
                // We are still playing
                update_environment(dt);
            }
        }
        break;
    case stSimulating:
        if (iPaused)
        {
            fTimeStarted += dt;
            fTimeToStop += dt;
            fTimeToPropagade += dt;
            break;
        }
        if (fTime >= fTimeToStop)
        {
            // STOP
            m_current_state = stStopped;
        }
        else
        {
            const u32 ptr =
                calc_cursor(fTimeStarted, fTime, get_length_sec(), p_source.freq, source()->data_info()); //--#SM+#--
            set_cursor(ptr);

            if (update_culling(dt))
            {
                // switch to: PLAY
                m_current_state = stPlaying;
                /*
                                u32 ptr						= calc_cursor(	fTimeStarted,
                                                                            fTime,
                                                                            get_length_sec(),
                                                                            source()->data_info());
                                set_cursor					(ptr);
                */
                SoundRenderCore->i_start(this);
                dispatch_prefill();
            }
        }
        break;
    case stPlayingLooped:
        if (iPaused)
        {
            stop_target();
            m_current_state = stSimulatingLooped;
            fTimeStarted += dt;
            fTimeToPropagade += dt;
            break;
        }
        if (!update_culling(dt))
        {
            // switch to: SIMULATE
            stop_target();
            m_current_state = stSimulatingLooped; // switch state
        }
        else
        {
            // We are still playing
            update_environment(dt);
        }
        break;
    case stSimulatingLooped:
        if (iPaused)
        {
            fTimeStarted += dt;
            fTimeToPropagade += dt;
            break;
        }
        if (update_culling(dt))
        {
            // switch to: PLAY
            m_current_state = stPlayingLooped; // switch state
            const u32 ptr =
                calc_cursor(fTimeStarted, fTime, get_length_sec(), p_source.freq, source()->data_info()); //--#SM+#--
            set_cursor(ptr);

            SoundRenderCore->i_start(this);
            dispatch_prefill();
        }
        break;
    }

    //--#SM+# Begin--
    // hard rewind
    switch (m_current_state)
    {
    case stStarting:
    case stStartingLooped:
    case stPlaying:
    case stSimulating:
    case stPlayingLooped:
    case stSimulatingLooped:
        if (fTimeToRewind > 0.0f)
        {
            const float fLength = get_length_sec();
            const bool bLooped = (fTimeToStop == 0xffffffff);

            R_ASSERT2(fLength >= fTimeToRewind, "set_time: target time is bigger than length of sound");

            const float fRemainingTime = (fLength - fTimeToRewind) / p_source.freq;
            const float fPastTime = fTimeToRewind / p_source.freq;

            fTimeStarted = fTime - fPastTime;
            fTimeToPropagade = fTimeStarted; //--> For AI events

            if (fTimeStarted < 0.0f)
            {
                Log("fTimer_Value = ", fTime);
                Log("fTimeStarted = ", fTimeStarted);
                Log("fRemainingTime = ", fRemainingTime);
                Log("fPastTime = ", fPastTime);
                R_ASSERT2(fTimeStarted >= 0.0f, "Possible error in sound rewind logic! See log.");

                fTimeStarted = fTime;
                fTimeToPropagade = fTimeStarted;
            }

            if (!bLooped)
            {
                //--> Пересчитываем время, когда звук должен остановиться [recalculate stop time]
                fTimeToStop = fTime + fRemainingTime;
            }

            const u32 ptr = calc_cursor(fTimeStarted, fTime, fLength, p_source.freq, source()->data_info());
            set_cursor(ptr);

            fTimeToRewind = 0.0f;
        }
    default: break;
    }
    //--#SM+# End--

    // if deffered stop active and volume==0 -> physically stop sound
    if (bStopping && fis_zero(fade_volume))
        i_stop();

    VERIFY2(!!(owner_data) || (!(owner_data) && (m_current_state == stStopped)), "owner");
    VERIFY2(owner_data ? *(int*)(owner_data->feedback) : 1, "owner");

    // footer
    bMoved = FALSE;
    if (m_current_state != stStopped)
    {
        if (fTime >= fTimeToPropagade)
            Event_Propagade();
    }
    else if (owner_data)
    {
        VERIFY(this == owner_data->feedback);
        owner_data->feedback = 0;
        owner_data = 0;
    }
}

IC void volume_lerp(float& c, float t, float s, float dt)
{
    const float diff = t - c;
    const float diff_a = _abs(diff);
    if (diff_a < EPS_S)
        return;
    float mot = s * dt;
    if (mot > diff_a)
        mot = diff_a;
    c += (diff / diff_a) * mot;
}

#include "xrServerEntities/ai_sounds.h"

bool CSoundRender_Emitter::update_culling(float dt)
{
    if (b2D)
    {
        occluder_volume = 1.f;
        fade_volume += dt * 10.f * (bStopping ? -1.f : 1.f);
    }
    else
    {
        // Check range
        const float dist = SoundRenderCore->listener_position().distance_to(p_source.position);
        if (dist > p_source.max_distance)
        {
            smooth_volume = 0;
            return FALSE;
        }

        // Calc attenuated volume
        float att = p_source.min_distance / (psSoundRolloff * dist);
        clamp(att, 0.f, 1.f);
        const float fade_scale = bStopping ||
                (att * p_source.base_volume * p_source.volume *
                        (owner_data->s_type == st_Effect ? psSoundVEffects * psSoundVFactor : psSoundVMusic) <
                    psSoundCull) ?
            -1.f :
            1.f;
        fade_volume += dt * 10.f * fade_scale;

        // Update occlusion
        const float occ = (owner_data->g_type == SOUND_TYPE_WORLD_AMBIENT) ?
            1.0f :
            scene->get_occlusion(p_source.position, .2f, occluder);
        volume_lerp(occluder_volume, occ, 1.f, dt);
        clamp(occluder_volume, 0.f, 1.f);
    }
    clamp(fade_volume, 0.f, 1.f);
    // Update smoothing
    smooth_volume = .9f * smooth_volume +
        .1f *
            (p_source.base_volume * p_source.volume *
                (owner_data->s_type == st_Effect ? psSoundVEffects * psSoundVFactor : psSoundVMusic) * occluder_volume *
                fade_volume);
    if (smooth_volume < psSoundCull)
        return FALSE; // allow volume to go up
    // Here we has enought "PRIORITY" to be soundable
    // If we are playing already, return OK
    // --- else check availability of resources
    if (target)
    {
        target->set_priority(priority());
        return TRUE;
    }
    return SoundRenderCore->i_allow_play(this);
}

float CSoundRender_Emitter::priority() const
{
    const float dist = SoundRenderCore->listener_position().distance_to(p_source.position);
    float att = p_source.min_distance / (psSoundRolloff * dist);
    clamp(att, 0.f, 1.f);
    return smooth_volume * att * priority_scale;
}

void CSoundRender_Emitter::update_environment(float dt)
{
    if (bMoved)
        e_target = *(CSoundRender_Environment*)scene->get_environment(p_source.position);
    e_current.lerp(e_current, e_target, dt);
}

void CSoundRender_Emitter::render()
{
    target->fill_parameters();
    if (target->get_Rendering())
        target->update();
    else
        target->render();
    dispatch_prefill();
}

void CSoundRender_Emitter::start(const ref_sound& _owner, u32 flags, float delay)
{
    const bool _loop = flags & sm_Looped;
    bIgnoringTimeFactor = flags & sm_IgnoreTimeFactor;
    starting_delay = delay;

    VERIFY(_owner);
    owner_data = _owner;
    VERIFY(owner_data);
    p_source.position.set(0, 0, 0);

    const auto info = source()->info();
    p_source.min_distance = info.min_dist;
    p_source.max_distance = info.max_dist;
    p_source.base_volume = info.base_volume;
    p_source.volume = 1.f;
    p_source.freq = 1.f;
    p_source.max_ai_distance = info.max_ai_dist;

    if (fis_zero(delay, EPS_L))
    {
        m_current_state = _loop ? stStartingLooped : stStarting;
    }
    else
    {
        m_current_state = _loop ? stStartingLoopedDelayed : stStartingDelayed;
        fTimeToPropagade =
            bIgnoringTimeFactor ? SoundRenderCore->TimerPersistent.GetElapsed_sec() : SoundRenderCore->Timer.GetElapsed_sec();
    }
    bStopping = FALSE;
    bRewind = FALSE;

    // Calc storage
    for (auto& buf : temp_buf)
        buf.resize(source()->data_info().bytesPerBuffer);

    ovf = source()->open();
}

void CSoundRender_Emitter::i_stop()
{
    bRewind = FALSE;
    if (target)
        stop_target();

    wait_prefill();
    if (owner_data)
    {
        source()->close(ovf);
        Event_ReleaseOwner();
        VERIFY(this == owner_data->feedback);
        owner_data->feedback = NULL;
        owner_data = NULL;
    }
    m_current_state = stStopped;
}

void CSoundRender_Emitter::stop(bool isDeffered)
{
    if (isDeffered)
        bStopping = TRUE;
    else
        i_stop();
}

void CSoundRender_Emitter::rewind()
{
    bStopping = FALSE;
    bRewind = TRUE;
}

void CSoundRender_Emitter::pause(bool bVal, int id)
{
    if (bVal)
    {
        if (0 == iPaused)
            iPaused = id;
    }
    else
    {
        if (id == iPaused)
            iPaused = 0;
    }
}

void CSoundRender_Emitter::cancel()
{
    // Msg		("- %10s : %3d[%1.4f] : %s","cancel",dbg_ID,priority(),source->fname);
    switch (m_current_state)
    {
    case stPlaying:
        stop_target();
        m_current_state = stSimulating;
        break;
    case stPlayingLooped:
        stop_target();
        m_current_state = stSimulatingLooped;
        break;
    default: VERIFY2(false, "Non playing ref_sound forced out of render queue"); break;
    }
}

void CSoundRender_Emitter::stop_target()
{
    wait_prefill();
    R_ASSERT1_CURE(target, { return; });
    target->stop();
    target = nullptr;
}
