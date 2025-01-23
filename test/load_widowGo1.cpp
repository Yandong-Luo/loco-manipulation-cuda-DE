#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <string>

// 回调函数：按下 ESC 键退出程序
void keyboard(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
}

int main() {
    // 加载 MJCF 文件
    std::string model_path = "../widowGo1/urdf/widowGo1.xml"; // 替换为你的 XML 文件路径
    char error[1000] = "Could not load model"; // 错误信息缓冲区
    mjModel* m = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));

    if (!m) {
        std::cerr << "Failed to load MJCF model from " << model_path << ": " << error << std::endl;
        return 1;
    }

    // 创建模拟器数据结构
    mjData* d = mj_makeData(m);

    // 初始化 GLFW
    if (!glfwInit()) {
        std::cerr << "Could not initialize GLFW." << std::endl;
        mj_deleteData(d);
        mj_deleteModel(m);
        return 1;
    }

    // 创建 OpenGL 上下文
    GLFWwindow* window = glfwCreateWindow(1200, 900, "MuJoCo Simulation", nullptr, nullptr);
    if (!window) {
        std::cerr << "Could not create GLFW window." << std::endl;
        glfwTerminate();
        mj_deleteData(d);
        mj_deleteModel(m);
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    glfwSetKeyCallback(window, keyboard);

    // 初始化 MuJoCo 可视化
    mjvScene scn;
    mjrContext con;
    mjv_defaultScene(&scn);
    mjv_makeScene(m, &scn, 1000);
    mjr_defaultContext(&con);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // 相机设置
    mjvCamera cam;
    mjv_defaultCamera(&cam);

    // 初始化 mjvOption
    mjvOption opt;
    mjv_defaultOption(&opt);

    // 主渲染循环
    while (!glfwWindowShouldClose(window)) {
        // 仿真一步
        mj_step(m, d);

        // 获取窗口大小
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);

        // 渲染场景
        mjrRect viewport = {0, 0, width, height};
        mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // 交换缓冲区并处理事件
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // 清理资源
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    glfwDestroyWindow(window);
    glfwTerminate();
    mj_deleteData(d);
    mj_deleteModel(m);

    return 0;
}
