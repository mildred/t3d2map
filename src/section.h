#ifndef SECTION_H
#define SECTION_H

#include <QObject>
#include <QIODevice>
#include <QMap>

typedef QMap<QString,QString> QStringMap;

class Section : public QObject
{
    Q_OBJECT
public:
    explicit Section(QObject *parent = 0);

    static Section *read_t3d(QIODevice &f);

protected:

    static QStringMap read_t3d_args(QString line);

signals:

public slots:

};

#endif // SECTION_H
