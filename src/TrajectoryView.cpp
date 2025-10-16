#include "TrajectoryView.h"
#include <QPainter>
#include <QPainterPath>   // ← 新增
#include <QtMath>
#include <algorithm>


TrajectoryView::TrajectoryView(QWidget* parent) : QWidget(parent) {
    setAutoFillBackground(true);
}

void TrajectoryView::appendPoint(const QPointF& xz) {
    pts_.push_back(xz);
    update();
}
void TrajectoryView::clear() {
    pts_.clear();
    update();
}

QPointF TrajectoryView::mapToView(const QPointF& p, const QRect& box,
                                  double xmin, double xmax, double zmin, double zmax) const
{
    // 保护：范围退化时给个最小跨度
    if (qFuzzyCompare(xmin, xmax)) { xmin -= 0.5; xmax += 0.5; }
    if (qFuzzyCompare(zmin, zmax)) { zmin -= 0.5; zmax += 0.5; }

    const double sx = box.width()  / (xmax - xmin);
    const double sz = box.height() / (zmax - zmin);

    // —— 与 OpenCV 图像坐标一致：x 右正，z **向下为正**（不翻转）
    const double vx = box.left() + (p.x() - xmin) * sx;
    const double vy = box.top()  + (p.y() - zmin) * sz;
    return {vx, vy};
}

void TrajectoryView::paintEvent(QPaintEvent*) {
    QPainter g(this);
    g.setRenderHint(QPainter::Antialiasing, true);

    // 背景
    g.fillRect(rect(), QColor(18, 18, 18));

    // 绘图区域
    QRect box = rect().adjusted(margin_, margin_, -margin_, -margin_);

    // 没有点就只画网格外框
    if (pts_.isEmpty()) {
        g.setPen(QColor(60,60,60));
        g.drawRect(box);
        return;
    }

    // 统计范围（给 10% padding）
    double xmin=pts_.first().x(), xmax=xmin, zmin=pts_.first().y(), zmax=zmin;
    for (const auto& p: pts_) {
        xmin = std::min(xmin, (double)p.x());
        xmax = std::max(xmax, (double)p.x());
        zmin = std::min(zmin, (double)p.y());
        zmax = std::max(zmax, (double)p.y());
    }
    const double dx = xmax - xmin, dz = zmax - zmin;
    if (dx > 1e-9) { xmin -= 0.1*dx; xmax += 0.1*dx; }
    if (dz > 1e-9) { zmin -= 0.1*dz; zmax += 0.1*dz; }

    // 网格（主/次）
    g.setPen(QPen(QColor(55,55,55), 1));
    g.drawRect(box);

    g.setPen(QPen(QColor(45,45,45), 1));
    for (int i=1;i<gridMajor_;++i) {
        const double t = double(i)/gridMajor_;
        // 竖线
        int x = box.left() + int(t * box.width());
        g.drawLine(x, box.top(), x, box.bottom());
        // 横线
        int y = box.top() + int(t * box.height());
        g.drawLine(box.left(), y, box.right(), y);
    }

    // 轨迹折线（绿色）
    QPainterPath path;
    QPointF p0 = mapToView(pts_.first(), box, xmin, xmax, zmin, zmax);
    path.moveTo(p0);
    for (int i=1;i<pts_.size();++i)
        path.lineTo(mapToView(pts_[i], box, xmin, xmax, zmin, zmax));

    g.setPen(QPen(QColor(60, 220, 60), 2));
    g.drawPath(path);
}
