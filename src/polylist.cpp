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
            obj->add_polygon(Polygon::read_t3d(f, l));
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
