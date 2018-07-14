#include "qtosgleftsidebar.h"
#include <iostream>
QtOSGLeftSideBar::QtOSGLeftSideBar(int widgetHeight,QWidget *parent):
    QWidget(parent),
    _pressedAction(NULL),
    _checkedAction(NULL),
    m_widgetHeight(widgetHeight)
{
    setFixedWidth(40);
    m_action_height = 40;
}

QtOSGLeftSideBar::~QtOSGLeftSideBar()
{

}

void QtOSGLeftSideBar::addAction(QAction *action)
{
//    action->setText("");
    _actions.push_back(action);
    update();
    return;
}

QAction *QtOSGLeftSideBar::addAction(const QString &text, const QIcon &icon)
{
    QAction *action = new QAction(icon, text, this);
//    action->setCheckable(true);
    _actions.push_back(action);
    update();
    return action;
}

void QtOSGLeftSideBar::paintEvent(QPaintEvent *event)
{

    QPainter p(this);
    p.eraseRect(event->rect());
    QFont fontText(p.font());
    fontText.setFamily("Helvetica Neue");
    p.setFont(fontText);

    //Adapter l'adresse de la texture ici
    QImage texture(":images/black.jpeg");
    p.fillRect(event->rect(), QBrush(texture));
    int actions_height = _actions.size()*m_action_height;

    int action_y = event->rect().height()/2-actions_height/2;
    foreach(QAction *action, _actions)
    {
        QRect actionRect(0, action_y, event->rect().width(), m_action_height);

//        if(action->isChecked())
//        {
//            p.setOpacity(0.5);
//            p.fillRect(actionRect, QColor(19, 19, 19));
//            p.setPen(QColor(9, 9, 9));
////            p.drawLine(actionRect.topLeft(), actionRect.topRight());
//            p.setOpacity(1);
//        }

//        if(action == _actions.last())
//        {
//            p.setPen(QColor(15, 15, 15));
//            p.setPen(QColor(55, 55, 55));

//        }

//        if(!action->isChecked())
//        {
//            p.setPen(QColor(15, 15, 15));
//            p.setPen(QColor(55, 55, 55));
//        }
//        int icon_size = 48;

        QRect actionIconRect(0, action_y, event->rect().width(), m_action_height-20);
        QIcon  actionIcon(action->icon());
        actionIcon.paint(&p, actionIconRect);

//        p.setPen(QColor(217, 217, 217));
//        if(action->isChecked())
//            p.setPen(QColor(255, 255, 255));
//        QRect actionTextRect(0, action_y+actionRect.height()-20, event->rect().width(), 15);
//        p.drawText(actionTextRect, Qt::AlignCenter, action->text());

        action_y += actionRect.height();
    }
}

void QtOSGLeftSideBar::mousePressEvent(QMouseEvent *event)
{
    _pressedAction = actionAt(event->pos());
    if(_pressedAction == NULL || _pressedAction == _checkedAction)
        return;
    emit _pressedAction->triggered(true);
    update();
}

//void QtOSGLeftSideBar::mouseReleaseEvent(QMouseEvent *event)
//{
//    QAction* tempAction = actionAt(event->pos());
//    if(_pressedAction != tempAction || tempAction == NULL)
//    {
//        _pressedAction = NULL;
//        return;
//    }
//    if(_checkedAction != NULL)
//        _checkedAction->setChecked(false);
//    _checkedAction = _pressedAction;
//    if(_checkedAction != NULL)
//        _checkedAction->setChecked(true);
//    update();
//    _pressedAction = NULL;
//    return;
//}

QSize QtOSGLeftSideBar::minimumSizeHint() const
{
    return QSize(90, _actions.size()*m_action_height);
}

QAction *QtOSGLeftSideBar::actionAt(const QPoint &at)
{
    int actions_height = _actions.size()*m_action_height;

    int action_y = rect().height()/2-actions_height/2;
    foreach(QAction *action, _actions)
    {
        QRect actionRect(0, action_y, rect().width(), m_action_height);
        if(actionRect.contains(at))
            return action;
        action_y += actionRect.height();
    }
    return NULL;
}
