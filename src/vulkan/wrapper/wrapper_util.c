#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "wrapper_util.h"

/* Only used when nothing above identifies the application. */
#ifndef WRAPPER_FALLBACK_APP_ID
#define WRAPPER_FALLBACK_APP_ID "app.gamenative"
#endif

static pthread_once_t imagefs_once = PTHREAD_ONCE_INIT;
static char imagefs_dir[PATH_MAX];
static char app_id[256];

/* Java package rules, loosely: dotted, [A-Za-z0-9_], starts with a letter. */
static int
valid_app_id(const char *s, size_t n)
{
   if (n < 3 || n >= sizeof(app_id))
      return 0;
   int dot = 0;
   for (size_t i = 0; i < n; i++) {
      char c = s[i];
      if (c == '.') {
         dot = 1;
         if (i == 0 || i == n - 1)
            return 0;
      } else if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                   (c >= '0' && c <= '9') || c == '_')) {
         return 0;
      }
   }
   return dot && ((s[0] >= 'a' && s[0] <= 'z') || (s[0] >= 'A' && s[0] <= 'Z'));
}

static int
app_id_from_cmdline(char *out, size_t size)
{
   char buf[512];
   int fd = open("/proc/self/cmdline", O_RDONLY | O_CLOEXEC);
   if (fd < 0)
      return 0;
   ssize_t n = read(fd, buf, sizeof(buf) - 1);
   close(fd);
   if (n <= 0)
      return 0;
   buf[n] = 0;
   char *colon = strchr(buf, ':');   /* "pkg:service" secondary processes */
   if (colon)
      *colon = 0;
   size_t len = strlen(buf);
   if (!valid_app_id(buf, len) || len >= size)
      return 0;
   memcpy(out, buf, len + 1);
   return 1;
}

/* /data/data/<id>/... or /data/user/<n>/<id>/... */
static int
app_id_from_data_path(const char *p, char *out, size_t size)
{
   const char *rest = NULL;
   if (!strncmp(p, "/data/data/", 11)) {
      rest = p + 11;
   } else if (!strncmp(p, "/data/user/", 11)) {
      rest = strchr(p + 11, '/');
      if (rest)
         rest++;
   }
   if (!rest)
      return 0;
   size_t len = strcspn(rest, "/:");
   if (!valid_app_id(rest, len) || len >= size)
      return 0;
   memcpy(out, rest, len);
   out[len] = 0;
   return 1;
}

/* First "<...>/imagefs" path element in a ':' separated string that exists. */
static int
imagefs_from_string(const char *s, char *out, size_t size)
{
   static const char key[] = "/imagefs";
   for (const char *hit = strstr(s, key); hit; hit = strstr(hit + 1, key)) {
      char next = hit[sizeof(key) - 1];
      if (next != '/' && next != ':' && next != 0)
         continue;
      const char *start = hit;
      while (start > s && start[-1] != ':')
         start--;
      if (*start != '/')
         continue;
      size_t len = (size_t)(hit - start) + sizeof(key) - 1;
      if (len >= size)
         continue;
      memcpy(out, start, len);
      out[len] = 0;
      struct stat sb;
      if (stat(out, &sb) == 0 && S_ISDIR(sb.st_mode))
         return 1;
   }
   return 0;
}

static void
imagefs_resolve(void)
{
   char id[sizeof(app_id)] = "";
   const char *e;

   if ((e = getenv("WRAPPER_IMAGEFS")) && e[0] == '/' &&
       strlen(e) < sizeof(imagefs_dir)) {
      strcpy(imagefs_dir, e);
      goto have_dir;
   }

   if ((e = getenv("WRAPPER_APP_ID")) && valid_app_id(e, strlen(e))) {
      snprintf(imagefs_dir, sizeof(imagefs_dir),
               "/data/data/%s/files/imagefs", e);
      goto have_dir;
   }

   /* Where this very library was loaded from. */
   Dl_info info;
   if (dladdr((void *)imagefs_resolve, &info) && info.dli_fname &&
       imagefs_from_string(info.dli_fname, imagefs_dir, sizeof(imagefs_dir)))
      goto have_dir;

   static const char *const env_keys[] = {
      "TMPDIR", "HOME", "LD_LIBRARY_PATH", "PREFIX", "PATH", "XDG_RUNTIME_DIR",
   };
   for (size_t i = 0; i < sizeof(env_keys) / sizeof(env_keys[0]); i++) {
      e = getenv(env_keys[i]);
      if (e && imagefs_from_string(e, imagefs_dir, sizeof(imagefs_dir)))
         goto have_dir;
   }

   /* Inside the Android app process itself the process name is the id. */
   if (app_id_from_cmdline(id, sizeof(id))) {
      snprintf(imagefs_dir, sizeof(imagefs_dir),
               "/data/data/%s/files/imagefs", id);
      goto have_dir;
   }

   snprintf(imagefs_dir, sizeof(imagefs_dir), "/data/data/%s/files/imagefs",
            WRAPPER_FALLBACK_APP_ID);

have_dir:
   if (!app_id_from_data_path(imagefs_dir, app_id, sizeof(app_id)) &&
       !app_id_from_cmdline(app_id, sizeof(app_id)))
      app_id[0] = 0;
}

const char *
wrapper_imagefs_dir(void)
{
   pthread_once(&imagefs_once, imagefs_resolve);
   return imagefs_dir;
}

const char *
wrapper_app_id(void)
{
   pthread_once(&imagefs_once, imagefs_resolve);
   return app_id[0] ? app_id : NULL;
}

char *
wrapper_imagefs_path(const char *sub)
{
   char *path = NULL;
   if (asprintf(&path, "%s/usr/%s", wrapper_imagefs_dir(), sub) < 0)
      return NULL;
   return path;
}

int
wrapper_mkdir_p(const char *path, mode_t mode)
{
   char tmp[PATH_MAX];
   size_t len = strlen(path);
   if (!len || len >= sizeof(tmp))
      return -1;
   memcpy(tmp, path, len + 1);
   mode_t old_mask = umask(0);
   for (char *p = tmp + 1; *p; p++) {
      if (*p != '/')
         continue;
      *p = 0;
      mkdir(tmp, mode);   /* EEXIST and parents we cannot create are fine */
      *p = '/';
   }
   int r = (mkdir(tmp, mode) == 0 || errno == EEXIST) ? 0 : -1;
   umask(old_mask);
   if (r == 0)
      chmod(tmp, mode);
   struct stat sb;
   return (stat(tmp, &sb) == 0 && S_ISDIR(sb.st_mode)) ? 0 : -1;
}
