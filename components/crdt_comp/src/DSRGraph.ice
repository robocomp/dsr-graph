module RoboCompDSR
{
    struct AttribValue
    {
        string type;
        string value;
        int length;
    }

    dictionary<string, AttribValue> Attribs;
    struct EdgeAttribs
	{ 
		string label;
		int from;
        int to;
		Attribs attrs; 
	};
    dictionary<int, EdgeAttribs> FanOut;

    // Topic for node graph sync
    struct Node
    { 
        string type;
        int id;
        Attribs attrs;
        FanOut fano;
    };

    // Topic for full graph requests
    struct GraphRequest
    {
        string from;
    };


    // --- Test: OrMap with AworSet

    struct PairInt {
        int first;
        int second;
    };

    dictionary<int, int> CausalContext;
    sequence< ::RoboCompDSR::PairInt> DotCloud;
    dictionary<::RoboCompDSR::PairInt, Node> DotKernelValue;


    struct DotContext {
        CausalContext cc;
        DotCloud dc;
    };

    struct DotKernel {
        DotKernelValue ds;
        DotContext cbase;
    }

    struct AworSet {
        int id;
        DotKernel dk; // Dot kernel
    };


    dictionary<int, AworSet> MapAworSet;

    struct OrMap {
        int id;
        MapAworSet m;
        DotContext cbase;
    }
}
