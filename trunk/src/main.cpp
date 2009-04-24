
#include <FL/Fl.H>
#include "ui.h"

#ifdef WIN32
#include <FL/x.H>
#include "resource.h"
#else
#ifdef __APPLE__

#else

#include <X11/xpm.h>
#include <FL/x.H>
#include "../icon.xpm"
#endif
#endif

int
main(int argc, char **argv) {

    UserInterface *ui = new UserInterface();
    Fl::scheme("plastic");
    Fl::visual(FL_DOUBLE|FL_INDEX);

#ifdef WIN32
    ui->mainWindow->icon((char *)LoadIcon(fl_display, MAKEINTRESOURCE(IDI_ICON)));
#else
#ifdef __APPLE__
    
    //No icon for mac yet.
    
#else
    fl_open_display();
    static Pixmap global_icon;
    static Pixmap global_mask;
    XpmCreatePixmapFromData(fl_display, DefaultRootWindow(fl_display),
                          (char**)icon_xpm,
                          &global_icon, &global_mask, NULL);
    ui->mainWindow->icon((char*)global_icon);
#endif
#endif

    glutInit(&argc, argv);
    ui->show(argc, argv);

    return Fl::run();
}
