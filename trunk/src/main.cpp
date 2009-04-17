
#include <FL/Fl.H>
#include "ui.h"

#ifdef WIN32
#include <FL/x.H>
#include "resource.h"
#endif

int
main(int argc, char **argv) {

    UserInterface *ui = new UserInterface();
    Fl::scheme("plastic");
    Fl::visual(FL_DOUBLE|FL_INDEX);

#ifdef WIN32
    ui->mainWindow->icon((char *)LoadIcon(fl_display, MAKEINTRESOURCE(IDI_ICON)));
#endif

    glutInit(&argc, argv);
    ui->show(argc, argv);

    return Fl::run();
}
