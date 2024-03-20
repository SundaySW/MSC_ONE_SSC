
struct task {
    void* context;
    void (*callback)(void*);
    void notify() {
        callback(context);
    }
};

struct client {
    static void handle(void* context) {
        static_cast<client*>(context)->handle();
    }
    void handle() {}
    void run_task() {
        task t = {this, &handle};
        scheduler.run(std::move(task));
    }
};
