module RoboCompDSR
{

    dictionary<string, string> Attribs;
    dictionary<int, Attribs> FanOut;
    struct Content
    { 
        string type;
        int id;
        Attribs attrs;
        FanOut fano;
    };
    dictionary<int, Content> DSRGraph;
   
}
