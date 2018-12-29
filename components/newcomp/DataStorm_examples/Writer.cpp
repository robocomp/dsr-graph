#include <DataStorm/DataStorm.h>


using namespace std;

int
main(int argc, char* argv[])
{
    cout << "Im working..." <<argc<<endl;
    try
    {
        DataStorm::Node node(argc, argv);

        DataStorm::Topic<string, string> topic(node, "hello");
        auto writer = DataStorm::makeSingleKeyWriter(topic, "foo");
        while(1)
        {
            topic.waitForReaders();

            writer.update("hello");

            topic.waitForNoReaders();

        }
    }
    catch(const std::exception& ex)
    {
        cerr << ex.what() << endl;
        return 1;
    }
    return 0;
}