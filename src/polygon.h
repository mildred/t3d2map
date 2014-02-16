#ifndef POLYGON_H
#define POLYGON_H

#include "section.h"
#include "vector.h"
#include "plane.h"

class Polygon : public Section
{
    Q_OBJECT

    Plane          plane;
    QList<double3> vertices;
    double3        textureU;
    double3        textureV;
    double2        texturePan;

public:
    explicit Polygon(QObject *parent = 0);

    static Polygon *read_t3d(QIODevice &f, QString line);

signals:

public slots:

};

#endif // POLYGON_H
