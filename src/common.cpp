#include <QCoreApplication>
#include <QDir>
#include "common.h"

QDir rootDir()
{
    QDir d(QCoreApplication::applicationDirPath());
    d.cdUp();
    return d;
}
