// TrajectoryView.cpp
#include "TrajectoryView.h"
#include <QPainter>
#include <QPen>

TrajectoryView::TrajectoryView(QWidget* p): QWidget(p) { setMinimumHeight(240); }

void TrajectoryView::appendPoint(const QPointF& p) { pts_.push_back(p); update(); }
void TrajectoryView::clear() { pts_.clear(); update(); }

void TrajectoryView::paintEvent(QPaintEvent*) {
    QPainter g(this);
    g.fillRect(rect(), Qt::black);
    g.setRenderHint(QPainter::Antialiasing,true);
    // 坐标系中心放在中下
    const QPointF origin(width()/2.0, height()-20.0);

    // 网格
    g.setPen(QPen(QColor(60,60,60),1));
    for (int x=0; x<width(); x+=50) g.drawLine(x,0,x,height());
    for (int y=0; y<height(); y+=50) g.drawLine(0,y,width(),y);

    // 轨迹
    g.setPen(QPen(Qt::green,2));
    QPointF prev = origin;
    for (auto &p : pts_) {
        QPointF q(origin.x() + p.x()*scale_, origin.y() - p.y()*scale_);
        g.drawLine(prev, q);
        prev = q;
    }
}
