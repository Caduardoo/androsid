package com.androsid

import android.util.Log
import java.io.BufferedOutputStream
import java.net.ServerSocket
import java.net.Socket
import java.util.concurrent.CopyOnWriteArrayList
import kotlin.concurrent.thread

class StreamServer(private val port: Int) {

    companion object {
        private const val TAG = "StreamServer"
    }

    private class Client(val socket: Socket) {
        val out = BufferedOutputStream(socket.getOutputStream(), 64 * 1024)
    }

    private val clients = CopyOnWriteArrayList<Client>()
    private var server: ServerSocket? = null
    @Volatile private var running = false

    fun start() {
        if (running) return
        running = true
        thread(name = "androsid-accept", isDaemon = true) {
            try {
                ServerSocket(port).also { server = it }.use { srv ->
                    Log.i(TAG, "listening on 0.0.0.0:$port")
                    while (running) {
                        val sock = srv.accept()
                        sock.tcpNoDelay = true
                        clients.add(Client(sock))
                        Log.i(TAG, "client connected: ${sock.inetAddress} (${clients.size} total)")
                    }
                }
            } catch (e: Exception) {
                if (running) Log.e(TAG, "accept loop died", e)
            }
        }
    }

    fun stop() {
        running = false
        try { server?.close() } catch (_: Exception) {}
        clients.forEach { try { it.socket.close() } catch (_: Exception) {} }
        clients.clear()
    }

    fun hasClients(): Boolean = clients.isNotEmpty()

    fun clientCount(): Int = clients.size

    fun broadcast(json: String) =
        broadcastLine((json + "\n").toByteArray(Charsets.UTF_8))

    fun broadcastLine(line: ByteArray) {
        if (clients.isEmpty()) return

        for (client in clients) {
            try {
                synchronized(client) {
                    client.out.write(line)
                    client.out.flush()
                }
            } catch (e: Exception) {
                Log.i(TAG, "client dropped: ${e.message}")
                clients.remove(client)
                try { client.socket.close() } catch (_: Exception) {}
            }
        }
    }
}
