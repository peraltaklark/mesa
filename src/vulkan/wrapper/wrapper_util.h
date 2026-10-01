#ifndef __WRAPPER_UTIL_H
#define __WRAPPER_UTIL_H

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#define CREATE_FOLDER(folder, mode) \
({ \
   struct stat sb; \
   if (stat(folder, &sb) != 0 || !S_ISDIR(sb.st_mode)) { \
      mode_t old_mask = umask(0); \
      mkdir(folder, (mode)); \
      umask(old_mask); \
      chmod(folder, (mode)); \
   } \
})

#ifdef __cplusplus
extern "C" {
#endif

char *
get_executable_name(void);

/* Android application id (package name) this process belongs to, or NULL when
 * it cannot be determined. Resolved once:
 *   WRAPPER_APP_ID, the imagefs path the wrapper was loaded from, the
 *   environment (TMPDIR/HOME/...), then the first word of /proc/self/cmdline. */
const char *
wrapper_app_id(void);

/* Root of the imagefs this wrapper runs in, e.g.
 * /data/data/<application id>/files/imagefs. Never NULL. Resolution order:
 *   WRAPPER_IMAGEFS, WRAPPER_APP_ID, the wrapper's own library path, the
 *   environment, /proc/self/cmdline, then WRAPPER_FALLBACK_APP_ID. */
const char *
wrapper_imagefs_dir(void);

/* <imagefs>/usr/<name>, formatted into a thread-safe malloc'd string. */
char *
wrapper_imagefs_path(const char *sub);

/* mkdir -p. Returns 0 when the directory exists afterwards. */
int
wrapper_mkdir_p(const char *path, mode_t mode);

#ifdef __cplusplus
}
#endif

#endif
