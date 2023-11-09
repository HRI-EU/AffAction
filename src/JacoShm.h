/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are
  met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#ifndef AFF_JACOSHM_H
#define AFF_JACOSHM_H

#include <Rcs_macros.h>
#include <Rcs_utils.h>

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <semaphore.h>
#include <unistd.h>


// define statement that sets the value from the returned argument of a function
#define JACO_SHM_NAME (buildPath2ShmPath()) //"/tmp/jaco + username + .shm"


/*
  0    j2s7s300_joint_1_L
  1    j2s7s300_joint_2_L
  2    j2s7s300_joint_3_L
  3    j2s7s300_joint_4_L
  4    j2s7s300_joint_5_L
  5    j2s7s300_joint_6_L
  6    j2s7s300_joint_7_L
  7    j2s7s300_joint_finger_1_L
  8    j2s7s300_joint_finger_2_L
  9    j2s7s300_joint_finger_3_L
*/
typedef struct
{
  double desPos[10];
  double currPos[10];
  double currVel[10];
  double currForce[6];
  double currLoopTime;
} KinovaArm;

typedef struct
{
  int count;
  KinovaArm right;
  KinovaArm left;
} KinovaShm;

struct KinovaShmRegion
{
  sem_t mutex;
  KinovaShm data;
} KinovaShmRegion;



// function that returns the location of the shared memory file,
// which is in /tmp folder and it is named jaco + username + .shm
char jaco_shm_name[150];
const char* buildPath2ShmPath()
{
  std::string userName = String_getEnv("USER");
  strcpy(jaco_shm_name, "/tmp/jaco_");
  strcat(jaco_shm_name, userName.c_str());
  strcat(jaco_shm_name, ".shm");
  return jaco_shm_name;
}


class JacoShm
{
public:

  JacoShm(const char* fileName) : data(NULL)//, creator(false)
  {
    int fd = open(fileName, O_RDWR | O_CREAT, 0660);
    RCHECK_MSG(fd!=-1, "error opening file \"%s\"", fileName);

    char readBuf[1024];
    ssize_t numBytesInShm = read(fd, (void*)&readBuf, 1024);

    bool createShm = numBytesInShm==0 ? true : false;

    if (!createShm)
    {
      RCHECK_MSG(numBytesInShm==sizeof(struct KinovaShmRegion),
                 "Shared memory size mismatch: file size is %zu, but struct has size %zu",
                 (size_t)numBytesInShm, (size_t)sizeof(struct KinovaShmRegion));
    }

    struct KinovaShmRegion* ptr;

    if (createShm)
    {
      RLOG(0, "***** Creating Shared Memory region *****");
      ssize_t nWritten = write(fd, &KinovaShmRegion, sizeof(struct KinovaShmRegion));
      RCHECK_MSG(nWritten==sizeof(struct KinovaShmRegion), "%zu != %zu",
                 (size_t)nWritten, (size_t)sizeof(struct KinovaShmRegion));
    }
    else
    {
      RLOG(0, "***** Connecting to existing Shared Memory region *****");
    }

    ptr = (struct KinovaShmRegion*) mmap(NULL, sizeof(struct KinovaShmRegion), PROT_READ | PROT_WRITE,
                                         MAP_SHARED, fd, 0);

    this->shm = ptr;
    this->data = &shm->data;
    close(fd);

    if (createShm)
    {
      if (sem_init(&shm->mutex, 1, 1) != 0)
      {
        RFATAL("sem_init error");
      }
    }

  }

  ~JacoShm()
  {
  }

  void lock()
  {
    sem_wait(&shm->mutex);
  }

  void unlock()
  {
    sem_post(&shm->mutex);
  }

  KinovaShm* data;

private:
  struct KinovaShmRegion* shm;
};




#endif   // AFF_JACOSHM_H
