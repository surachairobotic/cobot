#include "cobot_label.h"

CobotLabel::CobotLabel(QWidget* parent, Qt::WindowFlags f)
    : QLabel(parent) {

}

CobotLabel::~CobotLabel() {}

void CobotLabel::mousePressEvent(QMouseEvent* event) {
    Q_EMIT clicked();
}
