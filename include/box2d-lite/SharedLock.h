#include <omp.h>

class SharedLock {
private:
    omp_lock_t write_lock;   // Exclusivo para escritores
    omp_lock_t read_lock;    // Protege el contador de lectores
    int readers;

public:
    SharedLock() : readers(0) {
        omp_init_lock(&write_lock);
        omp_init_lock(&read_lock);
    }

    ~SharedLock() {
        omp_destroy_lock(&write_lock);
        omp_destroy_lock(&read_lock);
    }

    // Lock compartido (lectura)
    void lock_shared() {
        omp_set_lock(&read_lock);
        readers++;
        if (readers == 1) {
            // Primer lector bloquea a los escritores
            omp_set_lock(&write_lock);
        }
        omp_unset_lock(&read_lock);
    }

    void unlock_shared() {
        omp_set_lock(&read_lock);
        readers--;
        if (readers == 0) {
            // Último lector libera a los escritores
            omp_unset_lock(&write_lock);
        }
        omp_unset_lock(&read_lock);
    }

    // Lock exclusivo (escritura)
    void lock() {
        omp_set_lock(&write_lock);
    }

    void unlock() {
        omp_unset_lock(&write_lock);
    }
};
