#include <QRegExp>
#include <QDebug>
#include "polygon.h"
#include "vector.h"
#include "plane.h"
#include "polylist.h"

Polygon::Polygon(QObject *parent) :
    Section(parent)
{
}

Polygon *Polygon::read_t3d(QIODevice &f, QString line, PolyList *pl)
{
    static QRegExp end("^\\s*End Polygon");
    static QRegExp vector("^\\s*(\\S+)\\s+([\\S^,]+),([\\S^,]+),([\\S^,]+)");
    static QRegExp pan("^\\s*Pan\\s+U=([\\S]+)\\s+V=([\\S]+)");

    QStringMap args = read_t3d_args(line);

    Polygon *obj = new Polygon();
    while(true) {
        QString l = f.readLine();
        if(end.indexIn(l) == 0) {
            return obj;
        } else if(vector.indexIn(l) == 0) {
            QString vecname = vector.cap(1);
            double3 vec({vector.cap(2).toDouble(), vector.cap(3).toDouble(), vector.cap(4).toDouble()});

            if(vecname == "Origin")        obj->plane.origin = vec;
            else if(vecname == "Normal")   obj->plane.normal = vec;
            else if(vecname == "TextureU") obj->textureU = vec;
            else if(vecname == "TextureV") obj->textureV = vec;
            else if(vecname == "Vertex")   obj->vertices << pl->add_vertex(vec);

        } else if(pan.indexIn(l) == 0) {
            obj->texturePan = {vector.cap(1).toDouble(), vector.cap(2).toDouble()};
        } else {
            qDebug() << "unprocessed:" << l;
        }
    }
}
