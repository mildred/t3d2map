#include <QString>
#include <QFile>
#include <QDebug>

#define CATCH_CONFIG_RUNNER
#include "catch.hpp"

#include "polylist.h"

Catch::Session &get_catch_session()
{
    static Catch::Session test_session;
    return test_session;
}

TEST_CASE("Run the main program", "[main][.]")
{
    qDebug() << "Starting up...";

    QFile in;
    in.open(stdin, QIODevice::ReadOnly);
    PolyList *pl = PolyList::read_t3d(in, in.readLine());

    delete pl;
}

int main(int argc, char *argv[])
{
    Catch::Session &session = get_catch_session();

    // writing to session.configData() here sets defaults
    // this is the preferred way to set them

    if(argc > 1 && QString(argv[1]) == "test")
    {
        int returnCode = session.applyCommandLine( argc-1, argv+1 );
        if( returnCode != 0 ) // Indicates a command line error
            return returnCode;
    }
    else
    {
        session.configData().reporterName = "xml";
        session.configData().outputFilename = "t3d2map.xml";
        session.configData().testsOrTags.push_back("Run the main program");
    }

    // writing to session.configData() or session.Config() here
    // overrides command line args
    // only do this if you know you need to

    return session.run();
}
