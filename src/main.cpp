#include <iostream>
#include "agent/go1_agent.h"

int main()
{
    Go1Agent agent(ModelType::GO1_FLAT);
    agent.Run();
    return 0;
}
