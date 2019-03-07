pipeline {
  agent {
    node {
      label 'ubuntu'
    }

  }
  stages {
    stage('linux') {
      steps {
        sh '''sudo su
rm -rf evmbuild
'''
        sh '''cat /proc/cpuinfo
whoami'''
        sh 'mkdir -p evmbuild && cd evmbuild && sudo cmake $WORKSPACE -DLLVM_EXPERIMENTAL_TARGETS_TO_BUILD=EVM && sudo make -j 8'
        sh 'rm -rf evmbuild'
      }
    }
  }
}