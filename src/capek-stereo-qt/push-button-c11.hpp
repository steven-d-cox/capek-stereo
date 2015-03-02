
#pragma once

#include <QPushButton>


class QPushButtonC11 : public QPushButton
{

    Q_OBJECT
    ;

private:    
    QPushButtonC11(const QPushButtonC11&) = delete;
    void operator=(const QPushButtonC11&) = delete;

    std::function<void (bool)> * onClicked;

public:

    QPushButtonC11(QWidget * parent = 0) : QPushButton(parent), onClicked(nullptr) {}
    QPushButtonC11(const QString & text, QWidget * parent = 0) : QPushButton(text, parent), onClicked(nullptr) {}
    QPushButtonC11(const QIcon & icon, const QString & text, QWidget * parent = 0) : QPushButton(icon, text, parent), onClicked(nullptr) {}

    QPushButtonC11(std::function<void (bool)> onClicked_, QWidget * parent = 0) 
	: QPushButton(parent), onClicked(nullptr) { setOnClicked(onClicked_); }
    QPushButtonC11(const QString & text, std::function<void (bool)> onClicked_, QWidget * parent = 0)
	: QPushButton(text, parent), onClicked(nullptr) { setOnClicked(onClicked_); }
    QPushButtonC11(const QIcon & icon, const QString & text, std::function<void (bool)> onClicked_, QWidget * parent = 0)
	: QPushButton(icon, text, parent), onClicked(nullptr) { setOnClicked(onClicked_); }
    virtual ~QPushButtonC11() { delete onClicked; }

    void delOnClicked();
    void setOnClicked(std::function<void(bool)> f);

private slots:
    void handleClicked(bool checked);

};


