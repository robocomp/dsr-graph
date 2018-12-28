#include <DataStorm/DataStorm.h>

using namespace std;

int
main(int argc, char* argv[])
{
    try
    {
        DataStorm::Node node(argc, argv);
        DataStorm::Topic<string, string> topic(node, "hello");
        auto reader = DataStorm::makeSingleKeyReader(topic, "foo");
        auto sample = reader.getNextUnread();
        cout << sample.getKey() << " says " << sample.getValue() << "!" << endl;
    }
    catch(const std::exception& ex)
    {
        cerr << ex.what() << endl;
        return 1;
    }
    return 0;
}