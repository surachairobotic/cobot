#ifndef COBOT_LABEL_H
#define COBOT_LABEL_H

#include <QLabel>
#include <QWidget>
#include <Qt>

class CobotLabel : public QLabel {
    Q_OBJECT

public:
    explicit CobotLabel(QWidget* parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags());
    ~CobotLabel();

Q_SIGNALS:
    void clicked();

protected:
    void mousePressEvent(QMouseEvent* event);

};

#endif // COBOT_LABEL_H
