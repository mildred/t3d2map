#include <QDebug>
#include <QString>
#include <QRegExp>
#include "section.h"
#include "polygon.h"
#include "polylist.h"

Section::Section(QObject *parent) :
    QObject(parent)
{
}

Section *Section::read_t3d(QIODevice &f)
{
    static QRegExp begin("^\\s*Begin (\\S*)");
    QString l = f.readLine();
    if(begin.indexIn(l) != 0) {
        return nullptr;
    }

    QString name = begin.cap(1);

    if(name == "PolyList") {
        return PolyList::read_t3d(f, l);
    } else if(name == "Polygon") {
        return Polygon::read_t3d(f, l);
    } else {
        qDebug() << "unprocessed:" << l;
    }

    return nullptr;
}

QStringMap Section::read_t3d_args(QString line)
{
    static QRegExp keyval("(\\S+)=(\\S+)");
    QStringMap res;
    int offset = 0;

    while(offset = keyval.indexIn(line, offset), offset != -1) {
        res[keyval.cap(1)] = keyval.cap(2);
        offset++;
    }

    return res;
}
