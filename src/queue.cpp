//
// Created by anaple on 2024/4/25.
//

#include "queue.h"

CircularQueue::CircularQueue(int size) : maxSize(size), head(0), tail(0), isFull(false) {
    buffer.resize(size);
}

// --- 性能优化: 停止队列 ---
// 原因: 这个方法用于安全地停止队列操作。它设置停止标志并通知所有等待的线程，
// 这样它们就不会无限期地阻塞下去，从而确保程序可以干净利落地退出。
void CircularQueue::stop() {
    {
        std::unique_lock<std::mutex> lock(mutex_);
        stopped_ = true;
    }
    cond_not_empty_.notify_all();
    cond_not_full_.notify_all();
}

// --- 性能优化: 重写 enqueue 方法 ---
// 原因: 旧的实现是一个忙等待循环。新的实现使用互斥锁和条件变量。
// 当队列满时，生产者线程会调用 cond_not_full_.wait() 进入睡眠，释放CPU。
// 当消费者线程从队列中取出数据后，会调用 cond_not_full_.notify_one() 来唤醒一个生产者线程。
// 这样就避免了无效的CPU轮询。
bool CircularQueue::enqueue(const std::string& data) {
    std::unique_lock<std::mutex> lock(mutex_);
    cond_not_full_.wait(lock, [this] { return !isFull || stopped_; });

    if (stopped_) {
        return false;
    }

    buffer[tail] = data;
    tail = (tail + 1) % maxSize;
    isFull = tail == head;

    // 唤醒一个可能正在等待的消费者线程
    cond_not_empty_.notify_one();
    return true;
}

// --- 性能优化: 重写 dequeue 方法 ---
// 原因: 旧的实现也是一个忙等待循环。新的实现是阻塞式的。
// 当队列为空时，消费者线程会调用 cond_not_empty_.wait() 进入睡眠，释放CPU。
// 当生产者线程向队列中放入数据后，会调用 cond_not_empty_.notify_one() 来唤醒一个消费者线程。
// 同样，这避免了CPU资源的浪费。
bool CircularQueue::dequeue(std::string &data) {
    std::unique_lock<std::mutex> lock(mutex_);
    cond_not_empty_.wait(lock, [this] { return !isEmpty() || stopped_; });

    if (isEmpty() && stopped_) {
        return false;
    }

    data = buffer[head];
    head = (head + 1) % maxSize;
    isFull = false;

    // 唤醒一个可能正在等待的生产者线程
    cond_not_full_.notify_one();
    return true;
}


bool CircularQueue::isEmpty() const{
    // 这个函数在内部使用时会被互斥锁保护，所以本身不需要加锁
    return head == tail && !isFull;
}

int CircularQueue::getSize() const{
    // 这个函数在内部使用时会被互斥锁保护，所以本身不需要加锁
    return (tail - head + maxSize) % maxSize;
}
