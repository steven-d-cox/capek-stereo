
#include "stdinc.hpp"

#include "push-button-c11.hpp"

void QPushButtonC11::delOnClicked()
{
    if(onClicked == nullptr) return; 
    delete onClicked;
    disconnect(this, SIGNAL(clicked(bool)), this, SLOT(handleClicked(bool)));
}

void QPushButtonC11::setOnClicked(std::function<void(bool)> f)
{
    delOnClicked();
    onClicked = new std::function<void (bool)>(f);
    connect(this, SIGNAL(clicked(bool)), this, SLOT(handleClicked(bool)));
}

void QPushButtonC11::handleClicked(bool checked) 
{ 
    assert(onClicked != nullptr); 
    (*onClicked)(checked); 
}



