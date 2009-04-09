//#include "config.h"
#include <FL/Fl.H>
#include "ui.h"

int
main(int argc, char **argv) {

    UserInterface *ui = new UserInterface();
    Fl::scheme("plastic");
//Initial global objects.
    
    Fl::visual(FL_DOUBLE|FL_INDEX);

    ui->show(argc, argv);


    
    return Fl::run();
}
