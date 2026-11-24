/* Host regression for two device-manager callers running on different cores. */
#include <assert.h>
#include <errno.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <tk/tkernel.h>

static FastMLock device_lock;
static pthread_mutex_t state_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t flag_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t flag_changed = PTHREAD_COND_INITIALIZER;
static UINT flags;
static _Thread_local bool state_locked;
static atomic_uint active[2];
static unsigned acquisitions[2];

ER InitSpinLock(T_SPLOCK *lock)
{
    assert(lock == &device_lock.state_lock);
    return E_OK;
}

ER ISpinLock(T_SPLOCK *lock, UINT *saved)
{
    assert(lock == &device_lock.state_lock && !state_locked);
    assert(pthread_mutex_lock(&state_mutex) == 0);
    state_locked = true;
    *saved = 0;
    return E_OK;
}

ER ISpinUnlock(T_SPLOCK *lock, UINT saved)
{
    assert(lock == &device_lock.state_lock && state_locked && saved == 0);
    state_locked = false;
    assert(pthread_mutex_unlock(&state_mutex) == 0);
    return E_OK;
}

ID tk_cre_flg(const T_CFLG *config)
{
    assert(!state_locked && config->iflgptn == 0 && (config->flgatr & TA_WMUL));
    return 1;
}

ER tk_del_flg(ID id)
{
    assert(id == 1 && !state_locked);
    return E_OK;
}

ER tk_set_flg(ID id, UINT bits)
{
    assert(id == 1 && !state_locked);
    assert(pthread_mutex_lock(&flag_mutex) == 0);
    flags |= bits;
    assert(pthread_cond_broadcast(&flag_changed) == 0);
    assert(pthread_mutex_unlock(&flag_mutex) == 0);
    return E_OK;
}

ER tk_wai_flg(ID id, UINT bits, UINT mode, UINT *result, TMO timeout)
{
    assert(id == 1 && !state_locked && mode == (TWF_ORW | TWF_BITCLR));
    assert(pthread_mutex_lock(&flag_mutex) == 0);
    struct timespec deadline;
    assert(clock_gettime(CLOCK_REALTIME, &deadline) == 0);
    /* Bound infinite waits on the host so a lost wakeup fails the test. */
    unsigned wait_ms = timeout == TMO_FEVR ? 5000u : (unsigned)timeout;
    deadline.tv_nsec += (long)(wait_ms % 1000u) * 1000000L;
    deadline.tv_sec += wait_ms / 1000u + deadline.tv_nsec / 1000000000L;
    deadline.tv_nsec %= 1000000000L;
    int status = 0;
    while (!(flags & bits) && status == 0) {
        status = pthread_cond_timedwait(&flag_changed, &flag_mutex, &deadline);
    }
    if (status == 0) {
        *result = flags;
        flags &= ~bits;
    }
    assert(pthread_mutex_unlock(&flag_mutex) == 0);
    assert(status == 0 || status == ETIMEDOUT);
    return status == 0 ? E_OK : E_TMOUT;
}

static void *device_caller(void *first_lock)
{
    unsigned first = *(const unsigned *)first_lock;
    for (unsigned i = 0; i < 20000; ++i) {
        unsigned slot = (i + first) % 2u;
        INT lock_number = slot ? 31 : 0;
        assert(MLock(&device_lock, lock_number) == E_OK);
        assert(atomic_fetch_add(&active[slot], 1u) == 0u);
        ++acquisitions[slot];
        assert(atomic_fetch_sub(&active[slot], 1u) == 1u);
        assert(MUnlock(&device_lock, lock_number) == E_OK);
    }
    return NULL;
}

int main(void)
{
    assert(CreateMLock(&device_lock, (const UB *)"device manager") == E_OK);
    assert(MLock(&device_lock, 0) == E_OK);
    assert(MLock(&device_lock, 31) == E_OK);
    assert(MLockTmo(&device_lock, 0, 1) == E_TMOUT && device_lock.wai == 0);
    assert(MUnlock(&device_lock, 31) == E_OK);
    assert(MUnlock(&device_lock, 0) == E_OK);

    pthread_t callers[2];
    unsigned first[2] = {0, 1};
    for (unsigned i = 0; i < 2; ++i)
        assert(pthread_create(&callers[i], NULL, device_caller, &first[i]) == 0);
    for (unsigned i = 0; i < 2; ++i)
        assert(pthread_join(callers[i], NULL) == 0);
    assert(acquisitions[0] == 20000 && acquisitions[1] == 20000);
    assert(device_lock.wai == 0 && device_lock.flg == 0);
    assert(DeleteMLock(&device_lock) == E_OK);
    puts("PASS device-manager lock contention, independent bits, and bounded waits");
}
