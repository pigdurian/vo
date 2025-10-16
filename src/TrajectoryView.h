#pragma once
#include <QWidget>
#include <QVector>
#include <QPointF>

class TrajectoryView : public QWidget {
    Q_OBJECT
public:
    explicit TrajectoryView(QWidget* parent=nullptr);

    // 传入世界坐标平面点：x, z（单位：米）
    void appendPoint(const QPointF& xz);
    void clear();

protected:
    void paintEvent(QPaintEvent*) override;
    QSize minimumSizeHint() const override { return {360, 220}; }

private:
    QVector<QPointF> pts_;     // 存 (x,z)
    // 视图边距与网格
    int margin_ = 10;
    int gridMajor_ = 5;        // 5×5 主网格
    // 将 (x,z) 映射到像素（与 OpenCV 一致：y 向下）
    QPointF mapToView(const QPointF& xz, const QRect& box,
                      double xmin, double xmax, double zmin, double zmax) const;
};
