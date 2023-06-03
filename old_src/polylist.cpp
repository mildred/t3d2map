#include <algorithm>
#include <QRegExp>
#include <QDebug>
#include "polylist.h"
#include "polygon.h"

PolyList::PolyList(QObject *parent) :
    Section(parent)
{
}

PolyList* PolyList::read_t3d(QIODevice &f, QString line)
{
    static QRegExp end("^\\s*End PolyList");
    static QRegExp polygon("^\\s*Begin Polygon");

    QStringMap args = read_t3d_args(line);

    PolyList *obj = new PolyList();
    while(true) {
        QString l = f.readLine();
        if(end.indexIn(l) == 0) {
            return obj;
        } else if (polygon.indexIn(l) == 0) {
            obj->add_polygon(Polygon::read_t3d(f, l, obj));
        } else {
            qDebug() << "unprocessed:" << l;
        }
    }
}

void PolyList::add_polygon(Polygon *p)
{
    m_polys << p;
    p->setParent(this);
}

int PolyList::add_vertex(double3 v)
{
    auto res = std::find(m_vertices.begin(), m_vertices.end(), v);
    if(res == m_vertices.end()) return std::distance(m_vertices.begin(), res);

    int n = std::distance(m_vertices.begin(), m_vertices.end());
    m_vertices.push_back(v);
    return n;
}

double3 PolyList::vertex(int n)
{
    return m_vertices[n];
}
