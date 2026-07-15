#define _GNU_SOURCE

#include <dirent.h>
#include <dlfcn.h>
#include <limits.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

typedef void (*GlClearColorFn)(float red, float green, float blue, float alpha);
typedef void (*GlClearFn)(unsigned int mask);
typedef unsigned int (*EglSwapBuffersFn)(void *display, void *surface);

enum {
    GL_COLOR_BUFFER_BIT_VALUE = 0x00004000,
};

static pthread_once_t init_once = PTHREAD_ONCE_INIT;
static GlClearColorFn real_gl_clear_color;
static GlClearFn real_gl_clear;
static EglSwapBuffersFn real_egl_swap_buffers;
static int force_white_clear;

static int ov13855_sensor_is_bound(void)
{
    DIR *directory = opendir("/sys/bus/i2c/drivers/ov13855");
    struct dirent *entry;
    int found = 0;

    if (directory == NULL) {
        return 0;
    }
    while ((entry = readdir(directory)) != NULL) {
        const char *separator;

        if (entry->d_name[0] < '0' || entry->d_name[0] > '9') {
            continue;
        }
        separator = strchr(entry->d_name, '-');
        if (separator != NULL && strcmp(separator, "-0036") == 0) {
            found = 1;
            break;
        }
    }
    closedir(directory);
    return found;
}

static void initialize_shim(void)
{
    const char *enabled = getenv("XVERSE_WHITE_CAMERA_FALLBACK");
    char executable[PATH_MAX];
    const ssize_t length = readlink(
        "/proc/self/exe", executable, sizeof(executable) - 1);

    if (length > 0) {
        const char *name;

        executable[length] = '\0';
        name = strrchr(executable, '/');
        name = name == NULL ? executable : name + 1;
        force_white_clear = enabled != NULL
            && strcmp(enabled, "0") != 0
            && strcmp(name, "xverse_ar_engine") == 0;
    }

    *(void **)(&real_gl_clear_color) = dlsym(RTLD_NEXT, "glClearColor");
    *(void **)(&real_gl_clear) = dlsym(RTLD_NEXT, "glClear");
    *(void **)(&real_egl_swap_buffers) = dlsym(
        RTLD_NEXT, "eglSwapBuffers");
}

void glClearColor(float red, float green, float blue, float alpha)
{
    pthread_once(&init_once, initialize_shim);
    if (real_gl_clear_color == NULL) {
        return;
    }
    if (force_white_clear) {
        real_gl_clear_color(1.0f, 1.0f, 1.0f, 1.0f);
        return;
    }
    real_gl_clear_color(red, green, blue, alpha);
}

unsigned int eglSwapBuffers(void *display, void *surface)
{
    pthread_once(&init_once, initialize_shim);
    if (real_egl_swap_buffers == NULL) {
        return 0;
    }
    if (force_white_clear
            && real_gl_clear_color != NULL
            && real_gl_clear != NULL
            && !ov13855_sensor_is_bound()) {
        real_gl_clear_color(1.0f, 1.0f, 1.0f, 1.0f);
        real_gl_clear(GL_COLOR_BUFFER_BIT_VALUE);
    }
    return real_egl_swap_buffers(display, surface);
}
