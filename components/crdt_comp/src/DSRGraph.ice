module RoboCompDSR
{
    dictionary<string, string> Attribs;
    struct EdgeAttribs
	{ 
		string label;
		int from;
        int to;
		Attribs attrs; 
	};
    dictionary<int, EdgeAttribs> FanOut;

    // Topic for node graph sync
    struct Content
    { 
        string type;
        int id;
        Attribs attrs;
        FanOut fano;
        int causalContext;
        int dotCloud;
    };

    // Topic for full graph sync
    dictionary<int, Content> DSRGraph;

    // Topic for full graph requests
    struct GraphRequest
    {
        string from;
    };
}
