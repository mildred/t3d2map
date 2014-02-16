#ifndef POLYLIST_H
#define POLYLIST_H

#include <QList>
#include "section.h"

class Polygon;

class PolyList : public Section
{
    Q_OBJECT

    QList<Polygon*> m_polys;

public:
    explicit PolyList(QObject *parent = 0);

    static PolyList *read_t3d(QIODevice &f, QString line);

    void add_polygon(Polygon *p);

signals:

public slots:

};

#endif // POLYLIST_H
