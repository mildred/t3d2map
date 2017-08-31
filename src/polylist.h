#ifndef POLYLIST_H
#define POLYLIST_H

#include <vector>
#include <QList>
#include "section.h"
#include "vector.h"

class Polygon;

class PolyList : public Section
{
    Q_OBJECT

    QList<Polygon*>      m_polys;
    std::vector<double3> m_vertices;

public:
    explicit PolyList(QObject *parent = 0);

    static PolyList *read_t3d(QIODevice &f, QString line);

    void add_polygon(Polygon *p);

    int  add_vertex(double3 v);
    double3 vertex(int n);

signals:

public slots:

};

#endif // POLYLIST_H
