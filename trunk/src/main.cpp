
#include <FL/Fl.H>
#include "ui.h"

int
main(int argc, char **argv) {

    UserInterface *ui = new UserInterface();
    Fl::scheme("plastic");
    Fl::visual(FL_DOUBLE|FL_INDEX);

    glutInit(&argc, argv);
    ui->show(argc, argv);

    return Fl::run();
}
