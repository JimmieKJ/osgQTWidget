#ifndef QTOSGLEFTSIDEBAR_H
#define QTOSGLEFTSIDEBAR_H

#include <QWidget>
#include <QPainter>
#include <QPaintEvent>
#include <QIcon>
#include <QAction>

class QtOSGLeftSideBar: public QWidget
{
    Q_OBJECT
public:
    QtOSGLeftSideBar(int widgetHeight, QWidget *parent = 0);
    ~QtOSGLeftSideBar();
    void addAction(QAction *action);
    QAction *addAction(const QString &text, const QIcon &icon = QIcon());

protected:
    void paintEvent(QPaintEvent *event);
    void mousePressEvent(QMouseEvent *event);
//    void mouseReleaseEvent(QMouseEvent *event);
    QSize minimumSizeHint() const;

    QAction* actionAt(const QPoint &at);
private:
    QList<QAction*> _actions;
    QAction *_pressedAction;
    QAction *_checkedAction;
    int m_action_height;
    int m_widgetHeight;
    int m_widgetTopY;
};

#endif // QTOSGLEFTSIDEBAR_H
