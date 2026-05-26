#include <cstdio>

int main(int argc, char* argv[]) {
    const char* config_path = (argc > 1) ? argv[1] : "../configs/humanoid_lite.json";
    printf("humanoid_daemon starting — config: %s\n", config_path);
    printf("build OK\n");
    return 0;
}
