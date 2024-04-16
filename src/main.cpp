#include "agent/go1_agent.h"
#include "global_log.h"

int main()
{
    InitLogger();
    Go1Agent agent(ModelType::GO1_FLAT);
    agent.Run();
    return 0;
}