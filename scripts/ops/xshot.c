/* xshot.c —— 容器内 Xvfb 抓屏（无 ImageMagick/xwd 时的最小替代）
 * 编译：gcc -O2 -o xshot scripts/ops/xshot.c -lX11
 * 用法：DISPLAY=:99 ./xshot /tmp/shot.ppm   （PPM P6，宿主再用 convert 转 PNG）
 */
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <stdio.h>

int main(int argc, char **argv) {
    if (argc < 2) return 2;
    Display *display = XOpenDisplay(NULL);
    if (!display) return 3;
    Window root = DefaultRootWindow(display);
    XWindowAttributes attributes;
    if (!XGetWindowAttributes(display, root, &attributes)) return 4;
    XImage *image = XGetImage(display, root, 0, 0, attributes.width,
                              attributes.height, AllPlanes, ZPixmap);
    if (!image) return 5;
    FILE *stream = fopen(argv[1], "wb");
    if (!stream) return 6;
    fprintf(stream, "P6\n%d %d\n255\n", attributes.width, attributes.height);
    for (int y = 0; y < attributes.height; y++) {
        for (int x = 0; x < attributes.width; x++) {
            unsigned long pixel = XGetPixel(image, x, y);
            fputc((pixel >> 16) & 255, stream);
            fputc((pixel >> 8) & 255, stream);
            fputc(pixel & 255, stream);
        }
    }
    fclose(stream);
    XDestroyImage(image);
    XCloseDisplay(display);
    return 0;
}
