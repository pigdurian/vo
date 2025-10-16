// TrajectoryView.h
#pragma once
#include <QWidget>
#include <QVector>
#include <QPointF>

class TrajectoryView : public QWidget {
    Q_OBJECT
public:
    explicit TrajectoryView(QWidget* p=nullptr);
    void appendPoint(const QPointF& p);
    void clear();
protected:
    void paintEvent(QPaintEvent*) override;
private:
    QVector<QPointF> pts_;
    double scale_ = 10.0;  // 轨迹缩放
};
