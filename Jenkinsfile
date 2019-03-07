pipeline {
  agent {
    node {
      label 'ubuntu'
    }

  }
  stages {
    stage('linux') {
      steps {
        sh '''sudo rm -rf evmbuild
'''
        sh '''cat /proc/cpuinfo
'''
        sh 'mkdir -p evmbuild && cd evmbuild && sudo cmake $WORKSPACE -DLLVM_EXPERIMENTAL_TARGETS_TO_BUILD=EVM && sudo make -j 4'
        sh 'sudo rm -rf evmbuild'
      }
    }
  }
}